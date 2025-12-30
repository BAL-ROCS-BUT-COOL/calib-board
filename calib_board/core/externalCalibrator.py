from numpy.typing import NDArray

import numpy as np
import itertools
import cv2
from scipy.optimize import least_squares
import time 
import cProfile
import pstats
import copy
import matplotlib.pyplot as plt

from calib_commons.types import idtype
from calib_commons.utils import se3
from calib_commons.camera import Camera
from calib_commons.intrinsics import Intrinsics
from calib_commons.types import idtype
from calib_commons.utils.se3 import SE3
from calib_commons.world_frame import WorldFrame

from calib_board.core import ba 
from calib_board.core.correspondences import Correspondences, get_conform_views_of_cam, get_tracks, filter_with_track_length
from calib_board.core.checkerboard import CheckerboardMotion
from calib_board.core.observationCheckerboard import ObservationCheckerboard
from calib_board.core.scene import SceneCheckerboard, SceneType
from calib_board.core.checkerboard import Checkerboard
from calib_board.core.config import ExternalCalibratorConfig


class ExternalCalibrator: 
    """
    Manages the end-to-end process of extrinsic camera calibration using
    checkerboard observations.

    This class implements an incremental calibration strategy:
    1. It starts with an initial camera pair.
    2. It iteratively adds one camera at a time, estimating its pose using PnP.
    3. It discovers and adds new checkerboards to the scene.
    4. It refines all poses through bundle adjustment and filters out
       unreliable observations.
    """

    def __init__(
        self,
        correspondences: Correspondences,
        intrinsics: dict[idtype, Intrinsics],
        config: ExternalCalibratorConfig,
    ):
        """
        Initializes the ExternalCalibrator.

        Args:
            correspondences: A data structure holding all 2D checkerboard
                             observations for each camera.
            intrinsics: A dictionary mapping camera IDs to their intrinsic
                        parameters.
            config: A configuration object containing all calibration parameters.
        """
        # --- Input Data ---
        # A deepcopy is used to avoid modifying the original correspondences object.
        self.correspondences = copy.deepcopy(correspondences)
        self.intrinsics = intrinsics

        # --- Solving Parameters ---
        self.config = config
        self.free_first = False

        # --- Calibration State ---
        # The current estimate of all camera and checkerboard poses.
        self.estimate = SceneCheckerboard(scene_type=SceneType.ESTIMATE)
        # A list of checkerboards removed during filtering.
        self.checkers_removed: list[idtype] = []
        # A set of camera IDs that have not yet been added to the estimate.
        self.remaining_cameras: set[idtype] = set(self.correspondences.keys())
        

    def get_camera_score(self, cam_id: idtype) -> float:
        """
        Calculates a score for a camera based on the spatial distribution of
        its valid checkerboard observations. A higher score indicates that the
        observations are well-spread across the image plane, which is better
        for robust pose estimation.

        Args:
            cam_id: The ID of the camera to score.

        Returns:
            The calculated distribution score.
        """
        # Find checkerboards that are both visible by this camera and already in the scene.
        common_checkers_id = set(self.get_conform_views_of_cam(cam_id).keys()) & self.estimate.get_checkers_ids()
        # Further filter them to ensure they have been seen by enough cameras (long track).
        valid_common_checkers_id = self.filter_with_track_length(common_checkers_id)

        # Collect all 2D points from these valid views.
        image_points = []
        for checker_id in valid_common_checkers_id:
            image_points.append(
                self.correspondences[cam_id][checker_id].get_2d_conform()
            )

        # If there are any points, calculate the score; otherwise, the score is 0.
        if image_points:
            image_points_arr = np.vstack(image_points)
            score = self.view_score(image_points_arr, self.intrinsics[cam_id].resolution)
        else:
            score = 0.0

        return score


    def get_cameras_scores(self) -> dict[idtype, float]:
        """
        Calculates the distribution score for every camera currently in the scene estimate.

        Returns:
            A dictionary mapping each camera ID to its score.
        """
        return {
            cam_id: self.get_camera_score(cam_id)
            for cam_id in self.estimate.cameras.keys()
        }
    

    def calibrate(self) -> tuple[bool, dict[idtype, float]]:
        """
        Executes the main incremental calibration pipeline.

        Returns:
            A tuple containing:
            - A boolean indicating if the calibration was successful based on the
              final camera scores.
            - A dictionary of the final scores for each camera.
        """
        # 1. Initialization: Select and add the first camera.
        initial_camera_id = self.select_initial_camera()
        self.estimate.add_camera(
            Camera(initial_camera_id, SE3.idendity(), self.intrinsics[initial_camera_id])
        )
        if self.config.verbose >= 1:
            print(f"*********** Cam {initial_camera_id} added ***********")

        # Add all checkerboards visible by this first camera.
        self.add_checkers(initial_camera_id)
        self.remaining_cameras.remove(initial_camera_id)

        # 2. Incremental Step: Add remaining cameras one by one.
        while self.remaining_cameras:
            cam_id = self.select_next_best_cam()
            self.add_camera(cam_id)
            self.add_checkers(cam_id)
            self.iterative_filtering()

        if self.config.free_first:
            self.free_first = True
            print("Running BA with FREE first cam")
            self.iterative_filtering()

        # 3. Finalization and Reporting.
        print("\n##################### CALIBRATION TERMINATED #####################")
        print(f"Chessboards removed: {self.checkers_removed}")

        scores = self.get_cameras_scores()
        print(f"Camera observations scores: {scores}")

        # Check if all cameras meet the minimum score threshold.
        if min(scores.values()) > self.config.camera_score_threshold:
            print("Calibration Status: SUCCESS: each camera has sufficient distribution score.")
            return True, scores
        else:
            print("Calibration Status: FAILURE: some cameras do not have a sufficient distribution score.")
            return False, scores


    def get_scene(self, world_frame: WorldFrame) -> SceneCheckerboard:
        """
        Returns a copy of the scene estimate with all poses transformed to a
        new reference world frame.

        Args:
            world_frame: The desired world frame origin.

        Returns:
            A new SceneCheckerboard object with transformed poses.
        """
        if world_frame == WorldFrame.CAM_FIRST_CHOOSEN:
            T = np.eye(4)
        elif self.estimate.cameras.get(world_frame):
            T = se3.inv_T(self.estimate.cameras[world_frame].pose.mat)
        else:
            raise ValueError("World Frame not valid.")

        cameras = {
            id: Camera(id, SE3(T @ camera.pose.mat), camera.intrinsics)
            for id, camera in self.estimate.cameras.items()
        }
        checkers = {
            id: Checkerboard(
                id, SE3(T @ checker.pose.mat), self.config.checkerboard_geometry
            )
            for id, checker in self.estimate.checkers.items()
        }
        return SceneCheckerboard(
            cameras=cameras, checkers=checkers, scene_type=SceneType.ESTIMATE
        )


    def pnp(self, _2d: NDArray, _3d: NDArray, K: NDArray) -> NDArray:
        """
        Solves the Perspective-n-Point (PnP) problem to find the pose of a
        camera given 2D-3D point correspondences.

        Args:
            _2d: Array of 2D image points, shape (N, 2).
            _3d: Array of corresponding 3D object points, shape (N, 3).
            K: The 3x3 camera intrinsic matrix.

        Returns:
            The 4x4 camera pose matrix (T_W_C) that transforms points from the
            camera frame to the world frame.
        """
        # solvePnP finds the pose of the world frame relative to the camera (T_C_W).
        _, rvec, tvec = cv2.solvePnP(
            objectPoints=_3d, imagePoints=_2d, cameraMatrix=K, distCoeffs=None
        )

        T_C_W = se3.T_from_rvec_tvec(rvec, tvec)
        # We need the camera pose in the world frame, which is the inverse.
        T_W_C = se3.inv_T(T_C_W)
        return T_W_C


    def select_initial_camera(self) -> idtype:
        """
        Selects the best initial camera to start the calibration process.

        The "best" camera is the one that, when chosen as the origin, provides
        the most well-distributed view for another camera in the system. This
        ensures a strong geometric baseline for the initial estimate.

        Returns:
            The ID of the chosen initial camera.
        """
        cameras_id = list(self.correspondences.keys())
        arrangements = list(itertools.permutations(cameras_id, 2))

        best_score = -np.inf
        best_cam_id = None

        for cam0_id, cam1_id in arrangements:
            # Find common checkerboards between the two cameras.
            common_checkers_id = set(
                self.get_conform_views_of_cam(cam0_id).keys()
            ) & set(self.get_conform_views_of_cam(cam1_id).keys())
            valid_common_checkers_id = self.filter_with_track_length(
                common_checkers_id
            )

            # Score is based on the distribution of points in the second camera's view.
            image_points = []
            for checker_id in valid_common_checkers_id:
                image_points.append(self.correspondences[cam1_id][checker_id]._2d)

            if len(image_points):
                image_points_arr = np.vstack(image_points)
                score = self.view_score(
                    image_points_arr, self.intrinsics[cam1_id].resolution
                )
            else:
                score = 0

            score = self.view_score(image_points_arr, self.intrinsics[cam1_id].resolution)

            if score > best_score:
                best_score = score
                best_cam_id = cam0_id

        return best_cam_id
    

    def select_next_best_cam(self) -> idtype:
        """
        Selects the next best camera to add to the scene from the pool of
        remaining cameras. The "best" is the one with the highest observation
        score relative to the checkerboards already in the scene estimate.

        Returns:
            The ID of the selected camera.
        """
        best_score = -np.inf
        best_cam_id = None

        for cam_id in self.remaining_cameras:
            score = self.get_camera_score(cam_id)
            if score > best_score:
                best_score = score
                best_cam_id = cam_id

        return best_cam_id
    

    def view_score(self, image_points: NDArray, image_resolution: tuple[int, int]) -> float:
        """
        Computes a score based on the spatial distribution of points in an image.

        The image is divided into a multi-level grid. The score increases for
        each unique grid cell occupied by at least one point. Finer grids
        contribute more to the score, rewarding well-distributed points.

        Args:
            image_points: An array of 2D points, shape (N, 2).
            image_resolution: A tuple (width, height) of the image.

        Returns:
            The distribution score.
        """
        s = 0
        L = 3  # Number of grid levels.
        width, height = image_resolution

        for l in range(1, L + 1):
            K_l = 2**l  # Grid size for this level (e.g., 2x2, 4x4, 8x8).
            w_l = K_l  # Weight for this level.
            grid = np.zeros((K_l, K_l), dtype=bool)

            for point in image_points:
                if np.isnan(point).any():
                    continue

                # BUGFIX: apparently points can have coordinates that are slightly outside of (height, width) on rare occasions which leads to erroneous indices for the grid array!
                point[0] = np.clip(point[0], a_min=0, a_max=width)
                point[1] = np.clip(point[1], a_min=0, a_max=height)

                # Map the point to a grid cell index.
                x = int(np.ceil(K_l * point[0] / width)) - 1
                y = int(np.ceil(K_l * point[1] / height)) - 1

                # If the cell hasn't been visited yet, mark it and add to the score.
                if not grid[x, y]:
                    grid[x, y] = True
                    s += w_l
        return s
    

    def add_camera(self, cam_id: idtype) -> None:
        """
        Estimates the pose of a new camera and adds it to the scene.

        This method implements a RANSAC-like scheme:
        1. It iterates through each commonly observed checkerboard.
        2. For each, it computes an initial camera pose using PnP.
        3. It counts how many other checkerboard observations are "inliers"
           (i.e., consistent with this pose).
        4. It selects the pose that yielded the most inliers.
        5. It refines this best pose using all inlier points.

        Args:
            cam_id: The ID of the camera to add.
        """
        self.remaining_cameras.remove(cam_id)
        K = self.intrinsics[cam_id].K

        # Find checkerboards visible by this camera that are already in the scene.
        common_checkers_id = set(self.get_conform_views_of_cam(cam_id).keys()) & self.estimate.get_checkers_ids()
        valid_common_checkers_id = self.filter_with_track_length(common_checkers_id)

        # If there are no common views, we cannot position this camera yet.
        if not valid_common_checkers_id:
            return

        # RANSAC-like loop to find the best initial pose.
        best_num_inliers = 0
        best_inliers_mask = None
        best_cam_pose = None

        for checker_id in valid_common_checkers_id:
            # 1. Propose a model: Estimate pose from one checkerboard.
            obs = self.correspondences[cam_id][checker_id]
            valid_mask = ~np.isnan(obs._2d).any(axis=1)
            _2d = obs._2d[valid_mask]
            _3d = self.estimate.checkers[checker_id].get_3d_points_in_world()[valid_mask]
            
            # Not enough points to estimate a pose reliably.
            # if len(_2d) < 4:
            #     continue

            T_W_C_hyp = self.pnp(_2d, _3d, K)

            # 2. Score the model: Count inliers among all other common checkerboards.
            inliers_mask = []
            for checker_id_to_classify in valid_common_checkers_id:
                _3d_classify = self.estimate.checkers[checker_id_to_classify].get_3d_points_in_world()
                _2d_reprojection = self.intrinsics[cam_id].reproject(T_W_C_hyp, _3d_classify)
                _2d_observed = self.correspondences[cam_id][checker_id_to_classify]._2d

                errors = np.linalg.norm(_2d_observed - _2d_reprojection, axis=1)
                
                # A view is an inlier if enough of its points have low reprojection error.
                is_inlier = np.sum(errors < self.config.reprojection_error_threshold) >= \
                            self.config.min_number_of_valid_observed_points_per_checkerboard_view
                inliers_mask.append(is_inlier)

            # 3. Keep the best model found so far.
            num_inliers = np.sum(inliers_mask)
            if num_inliers > best_num_inliers:
                best_num_inliers = num_inliers
                best_inliers_mask = inliers_mask
                best_cam_pose = T_W_C_hyp
        
        # If no suitable pose was found (e.g., all PnP failed or had no inliers).
        if best_cam_pose is None:
            if self.config.verbose >= 1:
                print(f"!!! Could not add camera {cam_id}: No consistent pose found. !!!")
            return
            
        # 4. Refine the best pose using all points from the inlier views.
        _2d_all_inliers, _3d_all_inliers = [], []
        for i, checker_id in enumerate(list(valid_common_checkers_id)):
            if best_inliers_mask[i]:
                _2d_all_inliers.append(self.correspondences[cam_id][checker_id]._2d)
                _3d_all_inliers.append(self.estimate.checkers[checker_id].get_3d_points_in_world())
        
        _2d_refine = np.concatenate(_2d_all_inliers, axis=0)
        _3d_refine = np.concatenate(_3d_all_inliers, axis=0)
        
        # Initial guess for LM refinement is the best pose from RANSAC.
        T_C_W0 = se3.inv_T(best_cam_pose)
        rvec0, tvec0 = se3.rvec_tvec_from_T(T_C_W0)

        # Non-linear refinement using Levenberg-Marquardt.
        rvec_refined, tvec_refined = cv2.solvePnPRefineLM(
            objectPoints=_3d_refine,
            imagePoints=_2d_refine,
            cameraMatrix=K,
            distCoeffs=None,
            rvec=rvec0[:, np.newaxis],
            tvec=tvec0[:, np.newaxis],
        )
        T_C_W = se3.T_from_rvec_tvec(rvec_refined, tvec_refined)
        T_W_C = se3.inv_T(T_C_W)

        # 5. Add the refined camera to the scene.
        self.estimate.add_camera(Camera(cam_id, SE3(T_W_C), self.intrinsics[cam_id]))
        if self.config.verbose >= 1:
            if self.config.verbose >= 2:
                print(" ")
            print(f"*********** Cam {cam_id} added ***********")
            if self.config.verbose >= 2:
                print(f"had {len(valid_common_checkers_id)} views of checkers in common")
                print(f"initial pose refined on {best_num_inliers} checkers")
           

    def add_checkers(self, camera_id: idtype) -> None:
        """
        Identifies and adds new checkerboards to the scene that are visible
        from the given camera.

        Args:
            camera_id: The ID of the camera whose views should be inspected.
        """
        # Find checkerboards seen by this camera but not yet in the scene.
        new_checker_ids = set(self.get_conform_views_of_cam(camera_id).keys()) - self.estimate.get_checkers_ids()
        valid_checker_ids = self.filter_with_track_length(new_checker_ids)

        for checker_id in valid_checker_ids:
            obs = self.correspondences[camera_id][checker_id]
            valid_mask = ~np.isnan(obs._2d).any(axis=1)
            
            # Need a minimum number of points for PnP.
            # if np.sum(valid_mask) < 4:
            #     continue

            _2d = obs._2d[valid_mask]
            _3d = self.config.checkerboard_geometry._3d[valid_mask]

            # Pose of the checkerboard in the camera's frame (T_C_B)
            T_B_C = self.pnp(_2d, _3d, self.intrinsics[camera_id].K)
            
            # Transform to world frame: T_W_B = T_W_C * T_C_B
            T_W_C = self.estimate.cameras[camera_id].pose.mat
            T_W_B = T_W_C @ se3.inv_T(T_B_C)
            self.estimate.add_checker(
                Checkerboard(checker_id, SE3(T_W_B), self.config.checkerboard_geometry)
            )


    def iterative_filtering(self) -> None:
        """
        Performs iterative refinement of the scene by alternating between
        bundle adjustment and filtering of outlier observations.
        """
        if self.config.verbose >= 2:
            print("\n----- Iterative Filtering -----")
        
        iteration = 1
        continue_filtering = True
        while continue_filtering:
            if self.config.verbose >= 2:
                print(f"\n\n----- Iteration: {iteration} -----")
                print("** BA: **")
            iteration += 1

            self.bundle_adjustment()

            if self.config.verbose >= 2:
                print("\n** Filtering: **")
            
            continue_filtering = self.filtering()
           
  
    def filtering(self) -> bool:
        """
        Filters out observations with high reprojection errors and removes
        checkerboards that no longer have enough valid views.

        Returns:
            True if any view was filtered, False otherwise. This is used to
            control the iterative filtering loop.
        """
        num_views_filtered = 0
        checkers_removed_this_iter = []
        
        # Iterate over a copy of keys, as we may delete from the dictionary.
        checkers_ids = sorted(list(self.estimate.checkers.keys()))

        for camera in self.estimate.cameras.values():
            for checker_id in checkers_ids:
                checker = self.estimate.checkers.get(checker_id)
                if not checker:
                    continue  # Checker was already removed in this iteration.
                
                observation = self.correspondences[camera.id].get(checker.id)
                if not (observation and observation._is_conform):
                    continue
                
                # Calculate reprojection errors for this view.
                _2d_reprojection = camera.reproject(checker.get_3d_points_in_world())
                errors = np.linalg.norm(observation._2d - _2d_reprojection, axis=1)
                
                # Update the conformity mask for individual points.
                new_conformity_mask = errors < self.config.reprojection_error_threshold
                self.correspondences[camera.id][checker_id]._conformity_mask = new_conformity_mask
                
                # Check if the entire view is still considered valid.
                num_conform_points = np.sum(new_conformity_mask)
                is_view_conform = num_conform_points >= self.config.min_number_of_valid_observed_points_per_checkerboard_view
                
                # Only filter out non conform obs
                if is_view_conform:
                    continue
                
                num_views_filtered += 1
                self.correspondences[camera.id][checker.id]._is_conform = False
                
                # If removing this view makes the checker's track too short, remove the checker entirely.
                if len(self.get_tracks()[checker.id]) < self.config.min_track_length:
                    checkers_removed_this_iter.append(checker.id)
                    self.checkers_removed.append(checker.id)
                    del self.estimate.checkers[checker.id]

        if self.config.verbose >= 2:
            print(f" -> Number of views filtered: {num_views_filtered}")
            if checkers_removed_this_iter:
                print(f" -> Checkers removed from estimate: {sorted(list(set(checkers_removed_this_iter)))}")
        
        return num_views_filtered > 0
    

    def bundle_adjustment(self):
        """
        Performs bundle adjustment to jointly optimize all camera and
        checkerboard poses by minimizing the total reprojection error.
        """
        # 1. Convert current scene estimate into a flat parameter vector for the optimizer.
        x0 = self.parametrization_from_estimate(self.estimate, self.config.checkerboard_motion)

        t0 = time.time()
        
        # 2. Pack all required data into numpy arrays for the cost function.
        num_cameras = self.estimate.get_num_cameras()
        num_checkerboards = self.estimate.get_num_checkers()
        num_corners = self.config.checkerboard_geometry.get_num_corners()

        # Intrinsics array
        intrinsics_array = np.array([cam.intrinsics.K for cam in self.estimate.cameras.values()]).transpose(1, 2, 0)
        
        # Observations and indexing arrays
        observations_arr = np.full((2, num_cameras, num_checkerboards, num_corners), np.nan)
        cam_ids_map = {cam_id: i for i, cam_id in enumerate(self.estimate.cameras.keys())}
        chk_ids_map = {chk_id: k for k, chk_id in enumerate(self.estimate.checkers.keys())}

        for cam_id, i in cam_ids_map.items():
            for chk_id, k in chk_ids_map.items():
                obs = self.correspondences[cam_id].get(chk_id)
                if obs and obs._is_conform:
                    _2d_masked = obs._2d.T.copy()
                    _2d_masked[:, ~obs._conformity_mask] = np.nan
                    observations_arr[:, i, k, :] = _2d_masked
        
        # Flatten arrays and create a mask for valid (non-NaN) observations.
        observations_flat = observations_arr.ravel()
        valid_mask = ~np.isnan(observations_flat)
        observations = observations_flat[valid_mask]

        # Create corresponding flattened camera/checker indices for sparsity pattern.
        cam_indices = np.arange(num_cameras).reshape(1, -1, 1, 1)
        chk_indices = np.arange(num_checkerboards).reshape(1, 1, -1, 1)
        cameras_ids_obs = np.broadcast_to(cam_indices, observations_arr.shape).ravel()[valid_mask]
        checker_ids_obs = np.broadcast_to(chk_indices, observations_arr.shape).ravel()[valid_mask]

        # 3. Compute the Jacobian sparsity pattern to speed up optimization.
        A = ba.compute_jacobian_sparsity_pattern(
            len(observations), 
            cameras_ids_obs, 
            checker_ids_obs, 
            num_cameras, 
            num_checkerboards,
            self.free_first
        )

        # 4. Run the optimization.
        results = least_squares(
            fun=ba.cost_function_ba,
            x0=x0,
            jac_sparsity=A,
            method='trf',
            ftol=self.config.ba_least_square_ftol,
            x_scale='jac',
            verbose=self.config.least_squares_verbose,
            args=(
                num_cameras,
                num_checkerboards,
                self.config.checkerboard_motion,
                self.config.checkerboard_geometry._3d_augmented,
                intrinsics_array,
                observations,
                valid_mask,
                self.free_first
            ),
        )
               
        t1 = time.time()
        if self.config.verbose >= 2:
            print(f"BA optimization time: {t1 - t0:.2f} [s]")
        
        # 5. Update the scene estimate with the optimized parameters.
        self.estimate = self.update_estimate_from_parametrization(
            x=results.x,
            num_cameras=num_cameras,
            num_checkers=num_checkerboards,
        )
        
        
    def update_estimate_from_parametrization(
        self, x: NDArray, num_cameras: int, num_checkers: int
    ) -> SceneCheckerboard:
        """
        Updates the scene estimate (camera and checkerboard poses) from the
        optimized flat parameter vector.

        Args:
            x: The optimized parameter vector from `least_squares`.
            num_cameras: The number of cameras in the scene.
            num_checkers: The number of checkerboards in the scene.

        Returns:
            An updated SceneCheckerboard object.
        """
        camera_poses, checker_poses = ba.extract_all_poses_from_x(
            x, num_cameras, num_checkers, self.config.checkerboard_motion, self.free_first
        )
        
        estimate = copy.deepcopy(self.estimate)
        for i, camera_id in enumerate(estimate.cameras.keys()):
            estimate.cameras[camera_id].pose = SE3(camera_poses[:, :, i])

        for k, checker_id in enumerate(estimate.checkers.keys()):
            estimate.checkers[checker_id].pose = SE3(checker_poses[:, :, k])

        return estimate


    def parametrization_from_estimate(
        self, estimate: SceneCheckerboard, checkerboard_motion: CheckerboardMotion
    ) -> NDArray:
        """
        Converts the current scene estimate into a flat 1D numpy array (parameter
        vector) suitable for the `least_squares` optimizer.

        The vector is structured as:
        - Parameters for all cameras except the first (fixed) one.
        - Parameters for the absolute pose of the first checkerboard.
        - Parameters for the relative poses of all other checkerboards.

        Args:
            estimate: The current scene estimate.
            checkerboard_motion: The motion model, which determines the DoF
                                 for relative checkerboard poses.

        Returns:
            The flat 1D parameter vector.
        """
        params = []
        
        # Add parameters for cameras (skip the first, which is the fixed origin).
        camera_ids = list(estimate.cameras.keys())

        if not self.free_first:
            camera_ids = camera_ids[1:]

        for cam_id in camera_ids:
            params.append(se3.q_from_T(estimate.cameras[cam_id].pose.mat))
        
        # Add parameters for checkerboards.
        checker_ids = list(estimate.checkers.keys())
        first_checker_id = checker_ids[0]
        T_W_B1 = estimate.checkers[first_checker_id].pose
        
        # First checkerboard's pose is absolute (w.r.t. world).
        params.append(se3.q_from_T(T_W_B1.mat))

        # Other checkerboards' poses are relative to the first one.
        for chk_id in checker_ids[1:]:
            T_W_Bk = estimate.checkers[chk_id].pose
            T_B1_Bk = T_W_B1.inv().mat @ T_W_Bk.mat
            
            q = se3.q_from_T(T_B1_Bk)
            if checkerboard_motion == CheckerboardMotion.PLANAR:
                # For planar motion, only [theta_z, tx, ty] are optimized.
                q = q[[0, 3, 4]]
            params.append(q)
                
        return np.concatenate(params, axis=0) if params else np.array([])

    # getters

    def get_camera_ids(self) -> set[idtype]: 
        return set(self.correspondences.keys())

    def get_num_cameras(self) -> int: 
        return len(self.correspondences)
    
    def get_remaining_camera_ids(self) -> set[idtype]: 
        return self.get_camera_ids() - self.estimate.get_camera_ids()
    
    def get_conform_views_of_cam(self, cam_id) -> dict[idtype, ObservationCheckerboard]: 
        return get_conform_views_of_cam(cam_id, self.correspondences)

    def get_tracks(self) -> dict[idtype, set[idtype]]: 
       return get_tracks(self.correspondences)

    # manipulations

    def filter_with_track_length(self, checker_ids) -> set[idtype]: 
        return filter_with_track_length(self.correspondences, checker_ids, self.config.min_track_length)

    def print_errors(self): 
        for camera in self.estimate.cameras.values(): 
            for checker_id in sorted(list(self.estimate.checkers.keys())): # list to create a copy of the values so that when removing an element we do not raise an error.
                checker = self.estimate.checkers[checker_id]
                observation = self.correspondences[camera.id].get(checker.id)
                if observation and observation._is_conform:
                    _2d_reprojection = camera.reproject(checker.get_3d_points_in_world())
                    errors_xy = observation._2d - _2d_reprojection
                    errors = np.sqrt(np.sum(errors_xy**2, axis=1))
                    mean_error = np.mean(errors)
                    # if self.config.display_reprojection_errors:
                    #     print(f"view of checker {checker.id:>3} in cam {camera.id} error: {mean_error:.2f} [pix]")

    def display_histogram_reprojection_errors(self): 
        errors = self.estimate.reprojection_error_per_view(self.correspondences)
        for cam_id in errors.keys(): 
            error_for_this_cam = errors[cam_id]
            errors_values = list(error_for_this_cam.values())
            sorted_dict = sorted(error_for_this_cam.items(), key=lambda item: item[1])

            sorted_dict = dict(sorted_dict)
            print(f"cam {cam_id}")
            output = ""

            pad = len(str(max(sorted_dict.keys())))
            for key, value in sorted_dict.items():
                output += f"{key:<{pad}}: {value:.2f}, "

            # Remove the trailing comma and space from the last item
            output = output.rstrip(', ')
            print(output)

            plt.figure()
            plt.hist(errors_values, bins=30, color='blue', alpha=0.7)  # 'bins' defines how many sections to divide the data into
            plt.title(f"Error hist for cam {cam_id}")
            plt.xlabel('error')
            plt.ylabel('f')
