"""
This module defines the SceneGenerator class, which is responsible for creating
synthetic 3D scenes with cameras and checkerboards for testing and simulation
purposes.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

# Local application/library specific imports
from calib_board.core.checkerboard import Checkerboard, CheckerboardMotion
from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calib_board.core.scene import SceneCheckerboard, SceneType
from calib_commons.camera import Camera
from calib_commons.intrinsics import Intrinsics
from calib_commons.types import idtype
from calib_commons.utils.generateCircularCameras import generateCircularCameras
from calib_commons.utils.se3 import SE3, inv_T, T_from_rt
from calib_commons.utils.so3 import rot_Z


class SceneGenerator:
    """
    Generates synthetic scenes containing cameras and checkerboards.

    This class provides a way to procedurally create ground-truth scenes for
    evaluating calibration algorithms. It allows control over the number and
    layout of cameras, the number and motion of checkerboards, and the amount
    of noise in the final observations.
    """

    def __init__(
        self,
        num_checkerboards: int,
        checkerboard_geometry: CheckerboardGeometry,
        checkerboard_motion: CheckerboardMotion,
        checkerboard_motion_range: float,
        num_cameras: int,
        distance_cameras: float,
        tilt_cameras: float,
        min_track_length: int,
        noise_std: float,
    ) -> None:
        """
        Initializes the SceneGenerator with configuration parameters.

        Args:
            num_checkerboards: The total number of checkerboard poses to generate.
            checkerboard_geometry: The geometry of the checkerboards.
            checkerboard_motion: The motion model for the checkerboards
                                 (PLANAR or FREE).
            checkerboard_motion_range: The range for random translations of
                                       the checkerboards.
            num_cameras: The number of cameras to generate.
            distance_cameras: The distance of the cameras from the scene's center.
            tilt_cameras: The tilt angle of the cameras in degrees.
            min_track_length: The minimum number of cameras that must see a
                              checkerboard (used in downstream processing).
            noise_std: The standard deviation of Gaussian noise to add to the
                       synthetic 2D observations.
        """
        self.num_checkerboards = num_checkerboards
        self.checkerboard_geometry = checkerboard_geometry
        self.checkerboard_motion = checkerboard_motion
        self.checkerboard_motion_range = checkerboard_motion_range

        self.num_cameras = num_cameras
        self.distance_cameras = distance_cameras
        self.tilt_cameras = tilt_cameras

        self.min_track_length = min_track_length
        self.noise_std = noise_std

        # Default intrinsic parameters for generated cameras
        K = np.array(
            [[1055.0, 0.0, 1920.0 / 2.0], [0.0, 1055.0, 1080.0 / 2.0], [0.0, 0.0, 1.0]]
        )
        self.intrinsics = Intrinsics(K, (1920, 1080))

    def generate_scene(self) -> SceneCheckerboard:
        """
        Generates a complete synthetic scene with noisy observations.

        Returns:
            A `SceneCheckerboard` object representing the ground-truth scene,
            with the `reprojections` attribute populated with noisy 2D observations.
        """
        cameras, T_world_origin = self._generate_cameras(self.intrinsics)
        checkers = self._generate_checkers(T_world_origin)
        scene = SceneCheckerboard(
            cameras, checkers, scene_type=SceneType.SYNTHETIC
        )
        scene.generate_noisy_observations(self.noise_std)
        return scene

    def _generate_cameras(
        self, intrinsics: Intrinsics
    ) -> tuple[dict[idtype, Camera], np.ndarray]:
        """
        Generates a set of cameras arranged in a circle looking at the scene's center.

        Args:
            intrinsics: The intrinsic parameters to assign to each camera.

        Returns:
            A tuple containing:
            - A dictionary mapping camera IDs to `Camera` objects.
            - The transformation matrix of the first camera, used to define
              the world coordinate system.
        """
        point_to_look_at = np.append(
            self.checkerboard_geometry.get_center_checkerboard(), 0
        )
        poses = generateCircularCameras(
            point_to_look_at, self.distance_cameras, self.tilt_cameras, self.num_cameras
        )
        T_world_origin = poses[0]

        cameras = {
            i
            + 1: Camera(
                id=i + 1, pose=SE3(np.linalg.inv(T_world_origin) @ poses[i]), intrinsics=intrinsics
            )
            for i in range(self.num_cameras)
        }
        return cameras, T_world_origin

    def _generate_new_checkerboard(self, id: idtype, T_world_origin: np.ndarray) -> Checkerboard:
        """
        Generates a single new checkerboard with a randomly generated pose.

        Args:
            id: The ID to assign to the new checkerboard.
            T_world_origin: The transformation matrix that defines the origin of
                            the world frame (typically the first camera's pose).

        Returns:
            A new `Checkerboard` object.
        """
        # The first checkerboard is placed at the origin of the world.
        if id == 1:
            pose = inv_T(T_world_origin)
        else:
            if self.checkerboard_motion == CheckerboardMotion.PLANAR:
                alpha_deg = 360.0 * np.random.rand()
                R = rot_Z(alpha_deg)
                txy = self.checkerboard_motion_range * 2 * (np.random.rand(2) - 0.5)
                t = np.append(txy, 0)
            elif self.checkerboard_motion == CheckerboardMotion.FREE:
                eul_rad = 2 * np.pi * np.random.rand(3)
                R = Rotation.from_euler("zyx", eul_rad).as_matrix()
                t = self.checkerboard_motion_range * 2 * (np.random.rand(3) - 0.5)
            else:
                raise ValueError(f"Motion type '{self.checkerboard_motion}' is not valid.")

            pose = inv_T(T_world_origin) @ T_from_rt(R, t)

        return Checkerboard(id, SE3(pose), self.checkerboard_geometry)

    def _generate_checkers(
        self, T_world_origin: np.ndarray
    ) -> dict[idtype, Checkerboard]:
        """
        Generates a dictionary of all checkerboards for the scene.

        Args:
            T_world_origin: The transformation matrix defining the world origin.

        Returns:
            A dictionary mapping checkerboard IDs to `Checkerboard` objects.
        """
        return {
            i: self._generate_new_checkerboard(i, T_world_origin)
            for i in range(1, self.num_checkerboards + 1)
        }