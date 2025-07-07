"""
This module provides utility functions for processing and filtering calibration
correspondences. Correspondences are typically structured as a nested dictionary
mapping camera IDs to their observed checkerboards.

The functions herein allow for filtering based on observation quality, track
length (how many cameras see a given checkerboard), and other criteria.
"""

from __future__ import annotations

import numpy as np

from calib_board.core.observationCheckerboard import ObservationCheckerboard
from calib_commons.types import idtype

# A type alias for the main data structure:
# dict[camera_id, dict[checkerboard_id, ObservationCheckerboard]]
Correspondences = dict[idtype, dict[idtype, ObservationCheckerboard]]


def get_conform_views_of_cam(
    cam_id: idtype, correspondences: Correspondences
) -> dict[idtype, ObservationCheckerboard]:
    """
    Retrieves all "conform" checkerboard observations for a specific camera.

    An observation is conform if it meets certain quality criteria, typically
    meaning it was successfully and reliably detected.

    Args:
        cam_id: The ID of the camera to query.
        correspondences: The main dictionary of all observations.

    Returns:
        A dictionary mapping checkerboard IDs to their conform observations
        for the given camera. Returns an empty dictionary if the camera ID
        is not found or has no conform views.
    """
    if cam_id not in correspondences:
        return {}
    return {
        key: obs
        for key, obs in correspondences[cam_id].items()
        if obs._is_conform
    }


def get_tracks(correspondences: Correspondences) -> dict[idtype, set[idtype]]:
    """
    Computes the tracks for all checkerboards.

    A "track" for a checkerboard is the set of all camera IDs that have a
    conform observation of that board.

    Args:
        correspondences: The main dictionary of all observations.

    Returns:
        A dictionary where keys are checkerboard IDs and values are sets of
        camera IDs that observed them.
    """
    # First, find all unique checkerboard IDs across all observations
    all_checker_ids: set[idtype] = set()
    for obs_dict in correspondences.values():
        all_checker_ids.update(obs_dict.keys())

    # Initialize tracks for all known checkerboards
    tracks: dict[idtype, set[idtype]] = {
        chk_id: set() for chk_id in all_checker_ids
    }

    # Populate the tracks with camera IDs from conform observations
    for cam_id, observations in correspondences.items():
        for checker_id, observation in observations.items():
            if observation._is_conform:
                tracks[checker_id].add(cam_id)

    return tracks


def filter_with_track_length(
    correspondences: Correspondences,
    checker_ids: set[idtype],
    min_track_length: int,
) -> set[idtype]:
    """
    Filters a set of checkerboard IDs based on minimum track length.

    This function identifies which of the given checkerboard IDs are observed
    by at least a minimum number of cameras.

    Args:
        correspondences: The main dictionary of all observations.
        checker_ids: A set of checkerboard IDs to be filtered.
        min_track_length: The minimum number of cameras that must have observed
                          a checkerboard for it to be considered valid.

    Returns:
        A new set of checkerboard IDs that meet the minimum track length requirement.
    """
    tracks = get_tracks(correspondences)
    checkers_with_long_enough_track = {
        checker_id
        for checker_id, track in tracks.items()
        if len(track) >= min_track_length
    }
    return checker_ids & checkers_with_long_enough_track


def filter_correspondences_with_track_length(
    correspondences: Correspondences, min_track_length: int
) -> Correspondences:
    """
    Filters the entire correspondences structure by checkerboard track length.

    This removes all observations (from all cameras) of any checkerboard that
    is not seen by at least `min_track_length` cameras.

    Args:
        correspondences: The main dictionary of all observations.
        min_track_length: The minimum number of cameras required for a
                          checkerboard's observations to be kept.

    Returns:
        A new, filtered correspondences dictionary.
    """
    tracks = get_tracks(correspondences)
    valid_checker_ids = {
        checker_id
        for checker_id, track in tracks.items()
        if len(track) >= min_track_length
    }

    return {
        cam_id: {
            chk_id: obs
            for chk_id, obs in observations.items()
            if chk_id in valid_checker_ids
        }
        for cam_id, observations in correspondences.items()
    }


def filter_correspondences_with_non_nan_points(
    correspondences: Correspondences, threshold: int
) -> Correspondences:
    """
    Filters observations based on the number of valid (non-NaN) 2D points.

    This removes individual `ObservationCheckerboard` instances that do not have
    at least `threshold` valid corner detections.

    Args:
        correspondences: The main dictionary of all observations.
        threshold: The minimum number of non-NaN 2D points required for an
                   observation to be kept.

    Returns:
        A new, filtered correspondences dictionary.
    """
    return {
        cam_id: {
            chk_id: obs
            for chk_id, obs in observations.items()
            if np.sum(~np.isnan(obs._2d).any(axis=1)) >= threshold
        }
        for cam_id, observations in correspondences.items()
    }