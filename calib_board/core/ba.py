import scipy.sparse
import numpy as np

from numpy.typing import NDArray

from calib_board.core.checkerboard import CheckerboardMotion

def cost_function_ba(
    x: NDArray[np.float64],
    num_cameras: int,
    num_checkerboards: int,
    motion: CheckerboardMotion,
    _3dA: NDArray[np.float64],
    intrinsics_array: NDArray[np.float64],
    observations: NDArray[np.float64],
    mask: NDArray[np.bool_],
    free_first: bool
) -> NDArray[np.float64]:
    """
    Calculates the reprojection error vector for bundle adjustment.

    This function computes the difference between observed 2D points and
    reprojected 3D points based on the current estimates of camera and
    checkerboard poses.

    Args:
        x: A 1D array containing the optimization parameters (poses).
        num_cameras: The total number of cameras.
        num_checkerboards: The total number of checkerboards.
        motion: The motion type of the checkerboards (e.g., FREE, PLANAR).
        _3dA: Homogeneous coordinates of the checkerboard corners in its
              local frame (shape: 4xN_corners).
        intrinsics_array: A stack of intrinsic camera matrices
                          (shape: 3x3xN_cameras).
        observations: A 1D array of observed 2D corner points.
        mask: A boolean mask to select the valid reprojections that
              correspond to the observations.

    Returns:
        A 1D array of reprojection errors (residuals).
    """
    # Reproject 3D points onto all camera planes given the current pose estimates.
    reprojection_vectorized = compute_reprojection(
        x=x,
        num_cameras=num_cameras,
        num_checkers=num_checkerboards,
        motion=motion,
        _3dA=_3dA,
        intrinsics=intrinsics_array,
        free_first=free_first
    )

    # Flatten the reprojection results and apply the mask to get the valid estimates.
    valid_estimate = reprojection_vectorized.ravel()[mask]

    # The error is the difference between observed points and estimated projections.
    errors = observations - valid_estimate
    return errors


def compute_poses_from_6dof_vectorized(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Computes an array of 4x4 homogeneous transformation matrices from a
    vector of 6-DoF parameters (3 for rotation, 3 for translation) using
    vectorized operations.

    The rotation is parameterized by Z-Y-X Euler angles.

    Args:
        x: A flattened 1D numpy array of pose parameters. The size must be
           a multiple of 6. Shape: (N * 6,).

    Returns:
        An array of homogeneous transformation matrices. Shape: (4, 4, N).
    """
    num_poses = x.shape[0] // 6
    if x.shape[0] % 6 != 0:
        raise ValueError("Input vector size must be a multiple of 6.")

    # Reshape x to separate Euler angles and translation vectors.
    params = x.reshape(num_poses, 6)
    euler_angles = params[:, :3]
    translations = params[:, 3:]

    # Decompose Euler angles.
    Z, Y, X = euler_angles[:, 0], euler_angles[:, 1], euler_angles[:, 2]

    # Precompute sines and cosines for rotation matrices.
    cosZ, sinZ = np.cos(Z), np.sin(Z)
    cosY, sinY = np.cos(Y), np.sin(Y)
    cosX, sinX = np.cos(X), np.sin(X)

    # Create rotation matrices for each axis in a vectorized manner.
    # fmt: off
    Rz = np.stack([
        cosZ, -sinZ, np.zeros_like(Z),
        sinZ,  cosZ, np.zeros_like(Z),
        np.zeros_like(Z), np.zeros_like(Z), np.ones_like(Z)
    ], axis=1).reshape(num_poses, 3, 3)

    Ry = np.stack([
        cosY, np.zeros_like(Y),  sinY,
        np.zeros_like(Y), np.ones_like(Y), np.zeros_like(Y),
       -sinY, np.zeros_like(Y),  cosY
    ], axis=1).reshape(num_poses, 3, 3)

    Rx = np.stack([
        np.ones_like(X), np.zeros_like(X), np.zeros_like(X),
        np.zeros_like(X),  cosX, -sinX,
        np.zeros_like(X),  sinX,  cosX
    ], axis=1).reshape(num_poses, 3, 3)

    # Combine rotation matrices: R = Rz @ Ry @ Rx.
    # The @ operator handles batched matrix multiplication.
    R = Rz @ Ry @ Rx

    # Assemble the 4x4 homogeneous transformation matrices.
    poses = np.zeros((num_poses, 4, 4))
    poses[:, :3, :3] = R
    poses[:, :3, 3] = translations
    poses[:, 3, 3] = 1.0

    # Transpose from (N, 4, 4) to (4, 4, N) to match convention.
    return poses.transpose(1, 2, 0)


def compute_poses_from_3dof_vectorized(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Computes an array of 4x4 homogeneous transformation matrices from a
    vector of 3-DoF parameters (rotation about Z, translation in X and Y)
    using vectorized operations. This represents planar motion.

    Args:
        x: A flattened 1D numpy array of pose parameters. The size must be
           a multiple of 3. Shape: (N * 3,).

    Returns:
        An array of homogeneous transformation matrices. Shape: (4, 4, N).
    """
    num_poses = x.shape[0] // 3
    if x.shape[0] % 3 != 0:
        raise ValueError("Input vector size must be a multiple of 3.")

    # Reshape x to separate rotation angle and translation vector.
    params = x.reshape(num_poses, 3)
    theta_z = params[:, 0]
    translations_xy = params[:, 1:]

    # Precompute sines and cosines for the rotation matrix.
    cos_theta_z = np.cos(theta_z)
    sin_theta_z = np.sin(theta_z)

    # Create rotation matrices (Rz) in a vectorized manner.
    Rz = np.stack([
        cos_theta_z, -sin_theta_z, np.zeros_like(theta_z),
        sin_theta_z,  cos_theta_z, np.zeros_like(theta_z),
        np.zeros_like(theta_z), np.zeros_like(theta_z), np.ones_like(theta_z)
    ], axis=1).reshape(num_poses, 3, 3)

    # Assemble the 4x4 homogeneous transformation matrices.
    poses = np.zeros((num_poses, 4, 4))
    poses[:, :3, :3] = Rz
    poses[:, :2, 3] = translations_xy  # Set tx, ty
    poses[:, 3, 3] = 1.0

    # Transpose from (N, 4, 4) to (4, 4, N) to match convention.
    return poses.transpose(1, 2, 0)

def extract_all_poses_from_x(
    x: NDArray[np.float64],
    num_cameras: int,
    num_checkers: int,
    motion: CheckerboardMotion,
    free_first: bool
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    Extracts camera and checkerboard poses from the optimization vector 'x'.

    The structure of the vector 'x' depends on the `free_first` flag.

    If `free_first` is `True`, 'x' is structured as:
    1. Parameters for (num_cameras - 1) cameras (6 DoF each). The first
       camera is assumed to be at the identity pose.
    2. Parameters for the first checkerboard (6 DoF absolute pose).
    3. Parameters for (num_checkers - 1) relative checkerboard poses
       (6 or 3 DoF depending on the motion model).

    If `free_first` is `False`, 'x' is structured as:
    1. Parameters for all `num_cameras` cameras (6 DoF each).
    2. Parameters for the first checkerboard (6 DoF absolute pose).
    3. Parameters for (num_checkers - 1) relative checkerboard poses.

    Args:
        x: The 1D optimization vector.
        num_cameras: The total number of cameras.
        num_checkers: The total number of checkerboards.
        motion: The motion type of the checkerboards.
        free_first: If `True`, the first camera's pose is considered fixed at
                   the world origin (identity matrix) and its parameters are
                   omitted from `x`. If `False`, all camera poses are
                   treated as variables and are included in `x`.

    Returns:
        A tuple containing:
        - camera_poses: Array of camera poses (4, 4, num_cameras).
        - checker_poses: Array of checkerboard poses (4, 4, num_checkers).
    """
    # --- Extract Camera Poses ---
    camera_poses = np.zeros((4, 4, num_cameras))
    camera_poses[:, :, 0] = np.eye(4)  # First camera is fixed at the origin
    
    # Other cameras are optimized (num_cameras - 1 of them)
    cam_params_end_idx = num_cameras * 6 if free_first else (num_cameras - 1) * 6

    other_camera_poses = compute_poses_from_6dof_vectorized(x[:cam_params_end_idx])

    if free_first:
        camera_poses = other_camera_poses
    else:
        camera_poses[:, :, 1:] = other_camera_poses

    # --- Extract Checkerboard Poses ---
    checker_poses = np.zeros((4, 4, num_checkers))
    
    # The first checkerboard pose is absolute (6 DoF)
    first_checker_start_idx = cam_params_end_idx
    first_checker_end_idx = first_checker_start_idx + 6
    first_checker_pose_arr = compute_poses_from_6dof_vectorized(
        x[first_checker_start_idx:first_checker_end_idx]
    )
    # The result has shape (4, 4, 1), so we extract the single matrix
    first_checker_pose = first_checker_pose_arr[:, :, 0]
    checker_poses[:, :, 0] = first_checker_pose
    T_W_B1 = first_checker_pose

    # Other checkerboard poses are relative to the first one.
    other_checkers_params = x[first_checker_end_idx:]
    if motion == CheckerboardMotion.FREE:
        relative_poses = compute_poses_from_6dof_vectorized(other_checkers_params)
        other_checker_poses = np.einsum("ij,jkl->ikl", T_W_B1, relative_poses)
    elif motion == CheckerboardMotion.PLANAR:
        relative_poses = compute_poses_from_3dof_vectorized(other_checkers_params)
        other_checker_poses = np.einsum("ij,jkl->ikl", T_W_B1, relative_poses)
    else:
        raise ValueError(f"Unknown motion type {motion}.")

    if num_checkers > 1:
        checker_poses[:, :, 1:] = other_checker_poses

    return camera_poses, checker_poses

def compute_reprojection(
    x: NDArray[np.float64],
    num_cameras: int,
    num_checkers: int,
    motion: CheckerboardMotion,
    _3dA: NDArray[np.float64],
    intrinsics: NDArray[np.float64],
    free_first: bool
) -> NDArray[np.float64]:
    """
    Computes the reprojection of 3D points onto 2D image planes for all
    camera-checkerboard pairs.

    Args:
        x: The 1D optimization vector containing all pose parameters.
        num_cameras: The total number of cameras.
        num_checkers: The total number of checkerboards.
        motion: The motion type of the checkerboards.
        _3dA: Homogeneous coordinates of checkerboard corners in the checkerboard
              frame (shape: 4xN_corners).
        intrinsics: Stacked intrinsic matrices for all cameras
                    (shape: 3x3xN_cameras).

    Returns:
        An array of reprojected 2D points.
        Shape: (2, num_cameras, N_corners, num_checkers).
    """
    # 1. Reconstruct all camera and checkerboard poses from the parameter vector.
    camera_poses, checker_poses = extract_all_poses_from_x(
        x, num_cameras, num_checkers, motion, free_first=free_first
    )

    # 2. Transform checkerboard corner points to the world frame.
    # Operation: p_world = T_world_checker @ p_checker
    points_world = np.einsum("ijk,jl->ikl", checker_poses, _3dA)

    # 3. Compute inverse camera poses (world-to-camera transformation) efficiently.
    R = camera_poses[:3, :3, :]
    t = camera_poses[:3, 3, :]
    R_T = np.transpose(R, (1, 0, 2))
    inv_t = -np.einsum("ijn,jn->in", R_T, t)

    camera_poses_inv = np.zeros_like(camera_poses)
    camera_poses_inv[:3, :3, :] = R_T
    camera_poses_inv[:3, 3, :] = inv_t
    camera_poses_inv[3, 3, :] = 1.0

    # 4. Transform world points to each camera's coordinate frame.
    points_world_expanded = points_world[:, np.newaxis, :, :]
    camera_poses_inv_expanded = camera_poses_inv[:, :, :, np.newaxis]
    points_camera = np.einsum(
        "ijkl,jkmn->ikmn", camera_poses_inv_expanded, points_world_expanded
    )

    # 5. Project points from 3D camera coordinates to 2D image coordinates.
    points_camera_xyz = points_camera[:3, :, :, :]
    intrinsics_expanded = intrinsics[:, :, :, np.newaxis]
    projected_points_homogeneous = np.einsum(
        "ijkl,jklm->iklm", intrinsics_expanded, points_camera_xyz
    )

    # 6. Perform perspective divide to get final 2D coordinates.
    epsilon = 1e-8
    projected_points_2d = (
        projected_points_homogeneous[:2, :, :, :]
        / (projected_points_homogeneous[2, :, :, :] + epsilon)
    )
    return projected_points_2d


def compute_jacobian_sparsity_pattern(
    n_obs: int,
    camera_ids: NDArray[np.int_],
    checker_ids: NDArray[np.int_],
    n_cameras: int,
    n_checkers: int,
    free_first: bool
) -> scipy.sparse.csr_matrix:
    """
    Constructs the sparsity pattern of the Jacobian matrix for bundle adjustment.

    The Jacobian relates changes in pose parameters to changes in reprojection
    errors. An entry (i, j) is non-zero if the i-th observation depends on
    the j-th parameter. This implementation assumes all checkerboard poses
    are parameterized by 6-DoF, which may not be true for all motion types.

    Args:
        n_obs: Total number of observations (rows in the Jacobian).
        camera_ids: Array of camera indices for each observation.
        checker_ids: Array of checkerboard indices for each observation.
        n_cameras: Total number of cameras.
        n_checkers: Total number of checkerboards.
        free_first: Whether to fix the position of the first camera

    Returns:
        The Jacobian sparsity pattern as a SciPy CSR matrix.
    """
    PARAMS_PER_POSE = 6
    n_camera_params = n_cameras * PARAMS_PER_POSE if free_first else (n_cameras - 1) * PARAMS_PER_POSE
    n_checker_params = n_checkers * PARAMS_PER_POSE
    n_params = n_camera_params + n_checker_params

    row_indices, col_indices = [], []

    for obs_idx in range(n_obs):
        cam_idx = camera_ids[obs_idx]
        chk_idx = checker_ids[obs_idx]

        # Dependency on the camera pose (if not the fixed first camera).
        if not free_first and cam_idx > 0:
            cam_param_start = (cam_idx - 1) * PARAMS_PER_POSE
            row_indices.extend([obs_idx] * PARAMS_PER_POSE)
            col_indices.extend(range(cam_param_start, cam_param_start + PARAMS_PER_POSE))

        # Dependency on the absolute pose of the first checkerboard.
        first_checker_start = n_camera_params
        row_indices.extend([obs_idx] * PARAMS_PER_POSE)
        col_indices.extend(range(first_checker_start, first_checker_start + PARAMS_PER_POSE))

        # Dependency on the relative pose of the specific checkerboard observed.
        if chk_idx > 0:
            checker_param_start = (
                n_camera_params + PARAMS_PER_POSE + (chk_idx - 1) * PARAMS_PER_POSE
            )
            row_indices.extend([obs_idx] * PARAMS_PER_POSE)
            col_indices.extend(
                range(checker_param_start, checker_param_start + PARAMS_PER_POSE)
            )

    jacobian_sparsity = scipy.sparse.coo_matrix(
        (np.ones(len(row_indices)), (row_indices, col_indices)),
        shape=(n_obs, n_params),
    )

    return jacobian_sparsity.tocsr()
