from typing import Dict, Tuple

import numpy as np
from scipy.spatial.transform import Rotation 


from calib_commons.camera import Camera
from calib_commons.intrinsics import Intrinsics
from calib_commons.types import idtype

from calib_board.core.scene import SceneCheckerboard
from calib_board.core.scene import SceneType
from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calib_board.core.checkerboard import Checkerboard, CheckerboardMotion

from calib_commons.utils.generateCircularCameras import generateCircularCameras
from calib_commons.utils.se3 import *
from calib_commons.utils.so3 import *

class SceneGenerator: 

    def __init__(self, 
                 num_checkerboards: int, 
                 checkerboard_geometry: CheckerboardGeometry, 
                 checkerboard_motion: CheckerboardMotion,
                 checkerboard_motion_range: float, 
                 num_cameras: int, 
                 distance_cameras: float,
                 tilt_cameras: float, 
                 min_track_length: int, 
                 noise_std: float
                 ):
        
        self.numCheckerboards = num_checkerboards
        self.checkerboardGeometry = checkerboard_geometry
        self.checkerboardMotion = checkerboard_motion
        self.checkerboardMotionRange = checkerboard_motion_range

        self.numCameras = num_cameras
        self.distanceCameras = distance_cameras 
        self.tiltCameras = tilt_cameras

        self.minTrackLength = min_track_length
        self.noiseStd = noise_std
        
        K = np.array([[1055, 0, 1920/2], 
                      [0, 1055, 1080/2], 
                      [0, 0, 1]])
        self.intrinsics = Intrinsics(K, (1920, 1080))

    
    def generateScene(self) -> SceneCheckerboard: 
        cameras, T = self.generateCameras(self.intrinsics)
        checkers = self.generateCheckers(T)
        scene = SceneCheckerboard(cameras, checkers, scene_type=SceneType.SYNTHETIC)
        scene.generate_noisy_observations(self.noiseStd)
        return scene


    def generateCameras(self, intrinsics) -> Tuple[Dict[idtype, Camera], np.ndarray]: 
        pointToLookAt = np.append(self.checkerboardGeometry.get_center_checkerboard(), 0)
        poses = generateCircularCameras(pointToLookAt, self.distanceCameras, self.tiltCameras, self.numCameras)
        T = poses[0]  
        
        cameras = {}
        for i in range(self.numCameras):            
            pose = np.linalg.inv(T) @ poses[i]
            id = i+1
            cameras[id] = Camera(id, SE3(pose), intrinsics) 

        return cameras, T
    
    
    def generateNewCheckerboard(self, i, T) -> Checkerboard:
        if i == 1:
            pose = inv_T(T)  
        else:
            if self.checkerboardMotion == CheckerboardMotion.PLANAR:
                alpha = 360 * np.random.rand()
                R = rot_Z(alpha)
                txy = self.checkerboardMotionRange * 2 * (np.random.rand(2) - 0.5)
                t = np.append(txy, 0)
            elif self.checkerboardMotion == CheckerboardMotion.FREE:
                eul = 2 * np.pi * np.random.rand(3)
                # eul = np.array([0, 0,0])
                R =  Rotation.from_euler('zyx', eul).as_matrix()
                t = self.checkerboardMotionRange * 2 * (np.random.rand(3) - 0.5)
            else:
                raise ValueError("Motion type is not valid.")

            pose = inv_T(T) @ T_from_rt(R, t)  

        id = i
        checker = Checkerboard(id, SE3(pose), self.checkerboardGeometry) 
        return checker
        
    def generateCheckers(self, T) -> Dict[idtype, Checkerboard]: 
        return {id: self.generateNewCheckerboard(id, T) for id in range(1, self.numCheckerboards+1)}
            

