"""
This module defines the SceneCheckerboard class, which represents a 3D scene
composed of cameras and checkerboards. It is used to manage scene data,
generate synthetic observations, and calculate metrics like reprojection error.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

# Local application/library specific imports
from calib_board.core.checkerboard import Checkerboard
from calib_board.core.correspondences import Correspondences
from calib_board.core.observationCheckerboard import ObservationCheckerboard
from calib_board.utils.utils import is_chessboard_visible
from calib_commons.camera import Camera
from calib_commons.data.data_json import save_cameras_poses_to_json
from calib_commons.intrinsics import Intrinsics
from calib_commons.scene import SceneType
from calib_commons.types import idtype
from calib_commons.utils.se3 import q_from_T


class SceneCheckerboard:
    """
    Represents a 3D scene containing cameras and checkerboards.

    This class serves as a container for the geometric layout of a calibration
    setup. It can represent a ground-truth scene for simulation or an estimated
    scene during a calibration process. It also provides methods to generate
    synthetic observations and compute evaluation metrics.

    Attributes:
        cameras: A dictionary mapping camera IDs to Camera objects.
        checkers: A dictionary mapping checkerboard IDs to Checkerboard objects.
        type: The type of the scene (e.g., GROUND_TRUTH or ESTIMATE).
        reprojections: A correspondences dictionary containing synthetically
                       generated 2D observations.
    """

    def __init__(
        self,
        cameras: Optional[dict[idtype, Camera]] = None,
        checkers: Optional[dict[idtype, Checkerboard]] = None,
        scene_type: SceneType = SceneType.ESTIMATE,
    ) -> None:
        """
        Initializes a SceneCheckerboard instance.

        Args:
            cameras: An optional dictionary of Camera objects in the scene.
            checkers: An optional dictionary of Checkerboard objects in the scene.
            scene_type: The type of scene being represented.

        Raises:
            ValueError: If `scene_type` is not provided.
        """
        self.cameras: dict[idtype, Camera] = cameras if cameras is not None else {}
        self.checkers: dict[idtype, Checkerboard] = checkers if checkers is not None else {}
        self.type: SceneType = scene_type
        self.reprojections: Optional[Correspondences] = None

        if self.type is None:
            raise ValueError("Scene type must be provided.")

        # For an estimated scene, we might want to see the ideal reprojections
        if self.type == SceneType.ESTIMATE:
            self._generate_reprojections(
                noise_std=0.0, simulate_physical_constraints=False
            )

    def generate_noisy_observations(self, noise_std: float) -> None:
        """
        Generates synthetic, noisy 2D observations for the entire scene.

        This method populates the `reprojections` attribute with simulated
        checkerboard detections, adding Gaussian noise to the 2D points and
        simulating physical constraints like field-of-view and occlusion.

        Args:
            noise_std: The standard deviation of the Gaussian noise to add to
                       the 2D reprojection coordinates.

        Raises:
            ValueError: If called on a scene of type ESTIMATE.
        """
        if self.type == SceneType.ESTIMATE:
            raise ValueError(
                "Generating noisy observations should be done on a GROUND_TRUTH scene."
            )
        self._generate_reprojections(
            noise_std=noise_std, simulate_physical_constraints=True
        )

    def _generate_reprojections(
        self, noise_std: float, simulate_physical_constraints: bool
    ) -> None:
        """
        Internal method to generate reprojections for all camera-checker pairs.

        Args:
            noise_std: The standard deviation of Gaussian noise to add.
            simulate_physical_constraints: If True, checks for FOV and basic
                                           visibility constraints.
        """
        correspondences: Correspondences = {}
        for cam in self.cameras.values():
            correspondences[cam.id] = {}
            for checker in self.checkers.values():
                if simulate_physical_constraints and not is_chessboard_visible(
                    cam.pose.mat, checker.pose.mat
                ):
                    continue

                _3d_points = checker.get_3d_points_in_world()
                _2d_points = cam.reproject(_3d_points)
                _2d_points += noise_std * np.random.standard_normal(size=_2d_points.shape)

                if not simulate_physical_constraints:
                    correspondences[cam.id][checker.id] = ObservationCheckerboard(
                        _2d_points
                    )
                else:
                    within_fov_mask = cam.intrinsics.are_points_valid(_2d_points)
                    final_2d_points = np.full(_2d_points.shape, np.nan)
                    final_2d_points[within_fov_mask] = _2d_points[within_fov_mask]
                    correspondences[cam.id][checker.id] = ObservationCheckerboard(
                        final_2d_points
                    )
        self.reprojections = correspondences

    def get_intrinsics(self) -> dict[idtype, Intrinsics]:
        """
        Returns a dictionary of all camera intrinsics in the scene.

        Returns:
            A dictionary mapping camera IDs to their Intrinsics objects.
        """
        return {camera.id: camera.intrinsics for camera in self.cameras.values()}

    def get_correspondences(self) -> Optional[Correspondences]:
        """
        Returns the generated reprojections.

        Returns:
            The `Correspondences` dictionary, or None if not generated.
        """
        return self.reprojections

    def add_camera(self, camera: Camera) -> None:
        """Adds or updates a camera in the scene."""
        self.cameras[camera.id] = camera

    def add_checker(self, checker: Checkerboard) -> None:
        """Adds or updates a checkerboard in the scene."""
        self.checkers[checker.id] = checker

    def reprojection_error_per_view(
        self, observations: Correspondences
    ) -> dict[idtype, dict[idtype, float]]:
        """
        Calculates the mean reprojection error for each valid observation.

        Args:
            observations: The dictionary of observed 2D points.

        Returns:
            A nested dictionary mapping camera ID and then checkerboard ID to
            the mean reprojection error in pixels for that view.
        """
        reprojection_errors: dict[idtype, dict[idtype, float]] = {}
        for camera in self.cameras.values():
            reprojection_errors[camera.id] = {}
            for checker_id, checker in sorted(self.checkers.items()):
                observation = observations[camera.id].get(checker_id)

                # Only compute error for valid, conform observations
                if not (observation and observation._is_conform):
                    continue

                _2d_reprojection = camera.reproject(checker.get_3d_points_in_world())
                errors = np.linalg.norm(observation._2d - _2d_reprojection, axis=1)

                # Exclude NaNs from the mean calculation
                if np.all(np.isnan(errors)):
                    continue
                mean_error = np.nanmean(errors)
                reprojection_errors[camera.id][checker_id] = mean_error

        return reprojection_errors

    def get_camera_ids(self) -> set[idtype]:
        """Returns a set of all camera IDs in the scene."""
        return set(self.cameras.keys())

    def get_num_cameras(self) -> int:
        """Returns the total number of cameras in the scene."""
        return len(self.cameras)

    def get_checkers_ids(self) -> set[idtype]:
        """Returns a set of all checkerboard IDs in the scene."""
        return set(self.checkers.keys())

    def get_num_checkers(self) -> int:
        """Returns the total number of checkerboards in the scene."""
        return len(self.checkers)

    def print_cameras_poses(self, n_decimals: int = 2, spacing: int = 5) -> None:
        """
        Prints the poses of all cameras in a human-readable format.

        Args:
            n_decimals: The number of decimal places for printing.
            spacing: The spacing for formatting the output.
        """
        print("Camera poses:")
        for camera in self.cameras.values():
            q = q_from_T(camera.pose.mat)
            euler_deg = np.rad2deg(q[:3])
            translation = q[3:]

            t_str = (
                f"t = ({translation[0]:{spacing}.{n_decimals}f}, "
                f"{translation[1]:{spacing}.{n_decimals}f}, "
                f"{translation[2]:{spacing}.{n_decimals}f})"
            )
            e_str = (
                f"euler ZYX = ({euler_deg[0]:{spacing+2}.{n_decimals}f}, "
                f"{euler_deg[1]:{spacing+2}.{n_decimals}f}, "
                f"{euler_deg[2]:{spacing+2}.{n_decimals}f})"
            )
            print(f"    cam {camera.id}: {t_str}, {e_str}")

    def save_cameras_poses_to_json(self, path: str) -> None:
        """
        Saves the camera poses from the scene to a JSON file.

        Args:
            path: The file path where the JSON file will be saved.
        """
        save_cameras_poses_to_json(self.cameras, path)