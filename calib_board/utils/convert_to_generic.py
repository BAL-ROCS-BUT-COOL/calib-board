"""
This module provides utility functions to convert between scene and
correspondence representations specific to checkerboards and a more generic,
point-based representation used by other parts of the calibration library.
"""
from __future__ import annotations

import numpy as np

# Local application/library specific imports
from calib_board.core.correspondences import Correspondences as CheckerCorrespondences
from calib_board.core.scene import SceneCheckerboard
from calib_commons.objectPoint import ObjectPoint
from calib_commons.observation import Observation
from calib_commons.scene import Scene, SceneType
from calib_commons.types import idtype

# A type alias for the generic point-based correspondence structure
GenericCorrespondences = dict[idtype, dict[str, Observation]]


def convert_checker_scene_to_generic_scene(
    checker_scene: SceneCheckerboard, scene_type: SceneType
) -> Scene:
    """
    Converts a SceneCheckerboard into a generic, point-based Scene.

    This function "explodes" each checkerboard in the input scene into a
    collection of individual 3D `ObjectPoint` instances. Each new point gets a
    unique ID derived from its original checkerboard and its corner index.

    Args:
        checker_scene: The input scene containing checkerboard objects.
        scene_type: The scene type (e.g., GROUND_TRUTH, ESTIMATE) for the
                    newly created generic scene.

    Returns:
        A new `Scene` object where checkerboards are replaced by their
        constituent 3D corner points.
    """
    object_points = {
        f"{checker_id}_{i}": ObjectPoint(
            id=f"{checker_id}_{i}", position=_3d_corner
        )
        for checker_id, checker in checker_scene.checkers.items()
        for i, _3d_corner in enumerate(checker.get_3d_points_in_world())
    }

    return Scene(
        cameras=checker_scene.cameras,
        object_points=object_points,
        scene_type=scene_type,
    )


def convert_to_generic_correspondences(
    correspondences: CheckerCorrespondences,
) -> GenericCorrespondences:
    """
    Converts checkerboard-based correspondences to a generic point-based format.

    This function "explodes" each `ObservationCheckerboard`, which contains an
    array of 2D points, into individual `Observation` objects for each valid
    (non-NaN) corner point.

    An individual point observation is considered "conform" if and only if the
    parent `ObservationCheckerboard` is conform AND the specific point is an
    inlier according to the `_conformity_mask`.

    Args:
        correspondences: The input correspondence structure, mapping camera IDs
                         to dictionaries of observed checkerboards.

    Returns:
        A new correspondence structure where each observation is a single
        2D point, identified by a globally unique ID (e.g., "checkerID_cornerID").
    """
    # Using nested dictionary comprehensions for a concise and efficient conversion.
    return {
        cam_id: {
            f"{checker_id}_{i}": Observation(
                _2d=point_2d,
                is_conform=(
                    checker_obs._is_conform and checker_obs._conformity_mask[i]
                ),
            )
            for checker_id, checker_obs in obs_dict.items()
            if checker_obs._2d is not None
            for i, point_2d in enumerate(checker_obs._2d)
            if not np.isnan(point_2d).any()
        }
        for cam_id, obs_dict in correspondences.items()
    }