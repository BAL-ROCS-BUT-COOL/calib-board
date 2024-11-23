import scipy.sparse
import numpy as np

from calib_board.core.checkerboard import CheckerboardMotion

def cost_function_ba(x, 
                                num_cameras, 
                                num_checkerboards, 
                                motion, 
                                _3dA, 
                                intrinsics_array, 
                                observations, 
                                mask): 
    reprojection_vectorized = compute_reprojection(x=x, 
                                                   num_cameras=num_cameras, 
                                                   num_checkers=num_checkerboards, 
                                                   motion=motion, 
                                                   _3dA=_3dA, 
                                                   intrinsics=intrinsics_array)

    valid_estimate = reprojection_vectorized.ravel()[mask]
    errors = observations - valid_estimate
    # errors_vector = errors.ravel()
    return errors


def compute_poses_from_6dof_vectorized(x):
    """Compute the array of camera poses from the vector x using vectorized operations."""
    N = x.shape[0] // 6  # Number of cameras

    # Reshape x to separate Euler angles and translation vectors
    x = x.reshape(N, 6)
    euler_angles = x[:, :3]
    translations = x[:, 3:]

    # Compute rotation matrices
    Z = euler_angles[:, 0]
    Y = euler_angles[:, 1]
    X = euler_angles[:, 2]

    cosZ = np.cos(Z)
    sinZ = np.sin(Z)
    cosY = np.cos(Y)
    sinY = np.sin(Y)
    cosX = np.cos(X)
    sinX = np.sin(X)

    Rz = np.stack([cosZ, -sinZ, np.zeros_like(Z),
                   sinZ, cosZ, np.zeros_like(Z),
                   np.zeros_like(Z), np.zeros_like(Z), np.ones_like(Z)], axis=1).reshape(N, 3, 3)

    Ry = np.stack([cosY, np.zeros_like(Y), sinY,
                   np.zeros_like(Y), np.ones_like(Y), np.zeros_like(Y),
                   -sinY, np.zeros_like(Y), cosY], axis=1).reshape(N, 3, 3)

    Rx = np.stack([np.ones_like(X), np.zeros_like(X), np.zeros_like(X),
                   np.zeros_like(X), cosX, -sinX,
                   np.zeros_like(X), sinX, cosX], axis=1).reshape(N, 3, 3)

    # Combined rotation matrix R = Rz @ Ry @ Rx
    R = np.einsum('nij,njk,nkl->nil', Rz, Ry, Rx)

    # Initialize homogeneous transformation matrices
    poses = np.zeros((N, 4, 4))
    poses[:, :3, :3] = R
    poses[:, :3, 3] = translations
    poses[:, 3, 3] = 1.0

    # Transpose to (4, 4, N)
    return poses.transpose(1, 2, 0)

def compute_poses_from_3dof_vectorized(x):
    """Compute the array of camera poses from the vector x with 3 d.o.f. using vectorized operations."""
    N = x.shape[0] // 3  # Number of checkers

    # Reshape x to separate theta_z and translation vectors (x, y)
    x = x.reshape(N, 3)
    theta_z = x[:, 0]
    translations = x[:, 1:]

    # Compute rotation matrices
    cos_z = np.cos(theta_z)
    sin_z = np.sin(theta_z)

    Rz = np.stack([cos_z, -sin_z, np.zeros_like(theta_z),
                   sin_z, cos_z, np.zeros_like(theta_z),
                   np.zeros_like(theta_z), np.zeros_like(theta_z), np.ones_like(theta_z)], axis=1).reshape(N, 3, 3)

    # Initialize homogeneous transformation matrices
    poses = np.zeros((N, 4, 4))
    poses[:, :3, :3] = Rz
    poses[:, :2, 3] = translations
    poses[:, 3, 3] = 1.0

    # Transpose to (4, 4, N)
    return poses.transpose(1, 2, 0)

def extract_all_poses_from_x(x: np.ndarray, 
                             num_cameras: int, 
                             num_checkers: int,
                             motion): 
    
    camera_poses = np.zeros((4, 4, num_cameras))
    camera_poses[:,:,0] = np.eye(4)
    other_camera_poses = compute_poses_from_6dof_vectorized(x[:(num_cameras-1)*6])
    camera_poses[:,:,1:] = other_camera_poses
    
    checker_poses = np.zeros((4, 4, num_checkers))
    first_checker_pose = compute_poses_from_6dof_vectorized(x[(num_cameras-1)*6:(num_cameras)*6])
    checker_poses[:,:,0] = first_checker_pose[:,:,0]
    T_W_B1 = first_checker_pose[:,:,0]
    if motion == CheckerboardMotion.FREE:
        other_checker_poses = np.einsum('ij,jkl->ikl', T_W_B1, compute_poses_from_6dof_vectorized(x[(num_cameras)*6:]))
    elif motion == CheckerboardMotion.PLANAR: 
        other_checker_poses = np.einsum('ij,jkl->ikl', T_W_B1, compute_poses_from_3dof_vectorized(x[(num_cameras)*6:]))
    checker_poses[:,:,1:] = other_checker_poses
    return camera_poses, checker_poses

def compute_reprojection(x: np.ndarray, 
                         num_cameras: int,
                         num_checkers: int,
                         motion, 
                         _3dA: np.ndarray, 
                         intrinsics: np.ndarray):
    camera_poses, checker_poses = extract_all_poses_from_x(x, num_cameras, num_checkers, motion)
    points_world = np.einsum('ijk,jl->ikl', checker_poses, _3dA)

    camera_poses_inv = np.linalg.inv(camera_poses.transpose(2, 0, 1)).transpose(1, 2, 0) # to replace by invert by hand !!!!

    # Transform the points to the camera frame
    points_world_expanded = points_world[:, np.newaxis, :, :]  # Shape (4, 1, N_corners, N)
    camera_poses_inv_expanded = camera_poses_inv[:, :, :, np.newaxis]  # Shape (4, 4, N_cameras, 1)

    # Perform the transformation using einsum
    points_camera = np.einsum('ijkl,jkmn->ikmn', camera_poses_inv_expanded, points_world_expanded)
    points_camera_xyz = points_camera[:3, :, :, :]  # Shape (3, 4, N_corners, N)
    intrinsics_expanded = intrinsics[:, :, :, np.newaxis]  # Shape (3, 3, N_cameras, 1)
    projected_points_homogeneous = np.einsum('ijkl,jklm->iklm', intrinsics_expanded, points_camera_xyz)

    projected_points_2d = projected_points_homogeneous[:2, :, :, :] / projected_points_homogeneous[2, :, :, :]
    return projected_points_2d


def compute_jacobian_sparsity_pattern(n_obs, camera_ids, checker_ids, n_cameras, n_checkers):
            """
            Constructs the sparsity pattern of the Jacobian matrix for a given set of reprojection errors.
            
            Parameters:
            -----------
            n_obs : int
                Number of observation errors.
            camera_ids : np.ndarray of shape (n_obs,)
                Indices of the cameras corresponding to each observation.
            checker_ids : np.ndarray of shape (n_obs,)
                Indices of the checkerboards corresponding to each observation.
            n_cameras : int
                Total number of cameras (including the first fixed camera).
            n_checkers : int
                Total number of checkerboards.

            Returns:
            --------
            jacobian_sparsity : scipy.sparse.csr_matrix
                Sparsity pattern of the Jacobian matrix.
            """
            # Number of parameters in the vector x
            n_camera_params = (n_cameras - 1) * 6  # Each camera pose has 6 parameters (excluding the first camera)
            n_checker_params = 6  # First checkerboard pose
            n_other_checkers = (n_checkers - 1) * 6  # Other checkerboards (relative poses)

            n_params = n_camera_params + n_checker_params + n_other_checkers  # Total number of parameters

            # Initialize lists to construct the sparse matrix
            row_indices = []
            col_indices = []

            # Iterate over each observation
            for obs_idx in range(n_obs):
                # Each observation has:
                cam_idx = camera_ids[obs_idx]
                chk_idx = checker_ids[obs_idx]

                # If cam_idx is 0, the first camera is fixed, so no dependency.
                # Camera parameter index in x:
                if cam_idx > 0:  # Skip first camera (fixed)
                    camera_param_index = (cam_idx - 1) * 6
                    row_indices.extend([obs_idx] * 6)  # The observation depends on all 6 parameters
                    col_indices.extend(range(camera_param_index, camera_param_index + 6))

                # Dependency on the first checkerboard (always included)
                first_checker_index = n_camera_params  # First checkerboard starts right after camera parameters
                row_indices.extend([obs_idx] * 6)  # Each observation depends on 6 parameters of the first checkerboard
                col_indices.extend(range(first_checker_index, first_checker_index + 6))

                # Checkerboard parameter indices (relative pose)
                if chk_idx > 0:
                    # It depends on the other checkerboards (relative to the first)
                    checker_param_index = n_camera_params + 6 + (chk_idx - 1) * 6
                    row_indices.extend([obs_idx] * 6)  # Each observation depends on 6 parameters of this checkerboard
                    col_indices.extend(range(checker_param_index, checker_param_index + 6))

            # Construct sparse matrix
            jacobian_sparsity = scipy.sparse.coo_matrix(
                (np.ones(len(row_indices)), (row_indices, col_indices)),
                shape=(n_obs, n_params)
            )

            return jacobian_sparsity.tocsr()
