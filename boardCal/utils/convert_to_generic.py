import numpy as np

from boardCal.core.scene import SceneCheckerboard

from calibCommons.scene import Scene, SceneType
from calibCommons.objectPoint import ObjectPoint
from calibCommons.observation import Observation

def convert_checker_scene_to_generic_scene(checker_scene: Scene, scene_type):

    object_points = {}
    for checker_id, checker in checker_scene.checkers.items():
        _3d_corners = checker.get_3d_points_in_world()
        for i in range(len(_3d_corners)):
            global_id = f"{checker_id}_{i}"
            # position = _3d_corners[i,:]
            object_points[global_id] = ObjectPoint(global_id, _3d_corners[i,:])
       
    
    generic_scene = Scene(cameras = checker_scene.cameras, object_points=object_points, scene_type=scene_type)
    return generic_scene


def convert_to_generic_correspondences(correspondences):
    new_correspondences = {}
    for cam_id in correspondences:
        new_correspondences[cam_id] = {}
        for checker_id, checker_obs in correspondences[cam_id].items():
            _2d = correspondences[cam_id][checker_id]._2d
            # if checker_obs._is_conform:
            for i in range(len(_2d)):
                pt = _2d[i,:]
                if not np.isnan(pt).any():  # Check if neither coordinate is NaN
                    global_id = f"{checker_id}_{i}"
                    if checker_obs._is_conform and checker_obs._conformity_mask[i]:
                        is_conform = True
                    else: 
                        is_conform = False

                # is_conform = 
                    new_correspondences[cam_id][global_id] = Observation(_2d=pt, is_conform=is_conform)
            # if checker_obs._is_conform:
            #     for i in range(len(_2d)):
            #         pt = _2d[i,:]
            #         if not np.isnan(pt).any():  # Check if neither coordinate is NaN
            #             global_id = f"{checker_id}_{i}"
            #             is_conform = checker_obs._conformity_mask[i]
            #             new_correspondences[cam_id][global_id] = Observation(_2d=pt, is_conform=is_conform)
                
    return new_correspondences
