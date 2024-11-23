from typing import List, Dict
import numpy as np

from calibCommons.types import idtype
from calibCommons.utils.se3 import q_from_T
from calibCommons.camera import Camera
from calibCommons.intrinsics import Intrinsics
from calibCommons.scene import SceneType
from calibCommons.data.data_json import save_cameras_poses_to_json

from calib_board.core.checkerboard import Checkerboard
from calib_board.core.correspondences import Correspondences
from calib_board.core.observationCheckerboard import ObservationCheckerboard
from calib_board.utils.utils import is_chessboard_visible



class SceneCheckerboard:

    def __init__(self,
                 cameras: Dict[idtype, Camera] = None,
                 checkers: Dict[idtype, Checkerboard] = None,
                 scene_type: SceneType = None):

        if cameras is None:
            self.cameras = {}
        else:
            self.cameras = cameras
        if checkers is None:
            self.checkers = {}
        else:
            self.checkers = checkers

        self.type = scene_type

        self.reprojections: Correspondences = None

        if self.type is None:
            raise ValueError("scene type must be provided.")

        if self.type == SceneType.ESTIMATE:
            self.__generate_reprojections(0, False)

    def generate_noisy_observations(self, noise_std) -> None:
        if self.type == SceneType.ESTIMATE:
            raise ValueError("Not recommended to generate noisy observations on a scene estimate. You may instead want to generate exact reprojections.")

        # if self.reprojections is not None:
        #     raise ValueError("Noisy observations have already been generated.")

        self.__generate_reprojections(noise_std=noise_std, simulate_physical_constraints=True)

    def __generate_reprojections(self, noise_std, simulate_physical_constraints) -> None:
        correspondences = {} # dict by cam id

        for camera in self.cameras.values():
            correspondences[camera.id] = {}
            for checker in self.checkers.values():
                _2d = camera.reproject(checker.get_3d_points_in_world())
                _2d += noise_std * np.random.normal(size=(_2d.shape[0], 2))
                within_fov = camera.intrinsics.valid_points(_2d)
                if not simulate_physical_constraints:
                    correspondences[camera.id][checker.id] = ObservationCheckerboard(_2d)
                else:
                    if is_chessboard_visible(camera.pose.mat, checker.pose.mat):
                        _2dpoints = np.full(_2d.shape, np.nan)
                        _2dpoints[within_fov] = _2d[within_fov]
                        correspondences[camera.id][checker.id] = ObservationCheckerboard(_2dpoints)
                    
        self.reprojections = correspondences

    
    def get_intrinsics(self) -> Dict[idtype, Intrinsics]:
        return {camera.id: camera.intrinsics for camera in self.cameras.values()}

    def get_correspondences(self) -> Correspondences:
        return self.reprojections

    def add_camera(self, camera: Camera) -> None:
        if self.cameras is None:
            self.cameras = {}
        self.cameras[camera.id] = camera

    def add_checker(self, checker: Checkerboard) -> None:
        if self.checkers is None:
            self.checkers = {}
        self.checkers[checker.id] = checker

    def reprojection_error_per_view(self,
                                    observations: Correspondences) -> Dict[idtype, Dict[idtype, float]]:
        reprojection_errors = {}
        for camera in self.cameras.values():
            reprojection_errors[camera.id] = {}
            for checker_id in sorted(list(self.checkers.keys())):
                checker = self.checkers[checker_id]
                observation = observations[camera.id].get(checker.id)
                if observation and observation._is_conform:
                    _2d_reprojection = camera.reproject(checker.get_3d_points_in_world())
                    errors_xy = observation._2d - _2d_reprojection
                    errors = np.sqrt(np.sum(errors_xy**2, axis=1))
                    mean_error = np.mean(errors)
                    reprojection_errors[camera.id][checker.id] = mean_error

        return reprojection_errors

    def get_camera_ids(self) -> set[idtype]:
        return set(self.cameras.keys())

    def get_num_cameras(self) -> int:
        return len(self.cameras)

    def get_checkers_ids(self) -> set[idtype]:
        return set(self.checkers.keys())

    def get_num_checkers(self) -> int:
        return len(self.checkers)

    def print_cameras_poses(self, n_decimals=2, spacing=5) -> None:
        print(f"Camera poses: ")
        for camera in self.cameras.values():
            q = q_from_T(camera.pose.mat)
            euler = q[:3] * 180 / np.pi
            t = q[3:]

            print(f"    cam {camera.id}: t = ({t[0]:{spacing}.{n_decimals}f}, {t[1]:{spacing}.{n_decimals}f}, {t[2]:{spacing}.{n_decimals}f}), euler ZYX = ({euler[0]:{7}.{n_decimals}f}, {euler[1]:{7}.{n_decimals}f}, {euler[2]:{7}.{n_decimals}f})")

    def save_cameras_poses_to_json(self, path):
        save_cameras_poses_to_json(self.cameras, path)