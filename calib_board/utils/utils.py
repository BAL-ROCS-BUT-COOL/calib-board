"""
This module provides various utility functions related to checkerboard calibration,
including geometric visibility checks, data structure conversions, and video
frame extraction.
"""
import os
import subprocess
from typing import Optional

import numpy as np

# Local application/library specific imports
from calib_board.core.correspondences import Correspondences as CheckerCorrespondences
from calib_board.core.observationCheckerboard import ObservationCheckerboard
from calib_commons.types import idtype


def is_chessboard_visible(
    camera_pose: np.ndarray, chessboard_pose: np.ndarray
) -> bool:
    """
    Determines if a chessboard is visible from a camera based on orientation.

    This check is purely geometric and determines if the front face of the
    checkerboard is oriented towards the camera. It does not account for
    field of view or occlusion by other objects.

    Args:
        camera_pose: A 4x4 homogeneous transformation matrix representing the
                     camera's pose in the world frame (T_world_camera).
        chessboard_pose: A 4x4 homogeneous transformation matrix representing
                         the chessboard's pose in the world frame (T_world_board).

    Returns:
        True if the chessboard is facing the camera, False otherwise.
    """
    # The transformation from world to camera coordinates.
    T_camera_world = np.linalg.inv(camera_pose)

    # The pose of the chessboard in the camera's coordinate system.
    T_camera_board = T_camera_world @ chessboard_pose
    R_camera_board = T_camera_board[:3, :3]

    # The normal vector of the chessboard in its own local frame (positive Z-axis).
    board_normal_local = np.array([0.0, 0.0, 1.0])

    # The same normal vector transformed into the camera's coordinate system.
    board_normal_in_camera_frame = R_camera_board @ board_normal_local

    # For the board to be visible, its normal vector (when expressed in the
    # camera's frame) must point away from the camera's viewing direction.
    # In a standard CV camera frame, the viewing direction is +Z. The board's
    # normal must have a negative Z component to be "facing" the camera.
    return board_normal_in_camera_frame[2] < 0.0


def convert_correspondences_array_to_checker_correspondences(
    correspondences: dict[idtype, dict[idtype, np.ndarray]]
) -> CheckerCorrespondences:
    """
    Converts a dictionary of 2D point arrays into ObservationCheckerboard objects.

    Args:
        correspondences: A nested dictionary mapping camera ID and then
                         checkerboard ID to a numpy array of 2D points.

    Returns:
        A new correspondence structure where each numpy array is wrapped in an
        ObservationCheckerboard object.
    """
    return {
        cam_id: {
            checker_id: ObservationCheckerboard(points_2d=_2d)
            for checker_id, _2d in obs_dict.items()
        }
        for cam_id, obs_dict in correspondences.items()
    }


def extract_frames_from_video(
    video_path: str,
    output_dir: str,
    sampling_step: int,
    start_time: Optional[str] = None,
    end_time: Optional[str] = None,
) -> None:
    """
    Extracts frames from a video file using the ffmpeg command-line tool.

    This function attempts to use CUDA hardware acceleration if available to
    speed up the process.

    Args:
        video_path: The path to the input video file.
        output_dir: The directory where the extracted frames will be saved.
        sampling_step: The frame sampling rate (e.g., 5 means every 5th frame).
        start_time: The optional start time for extraction (format: "hh:mm:ss.ms").
        end_time: The optional end time for extraction (format: "hh:mm:ss.ms").
    """
    # Build time-trimming argument string
    trim_txt = ""
    if start_time:
        trim_txt += f"-ss {start_time} "
    if end_time:
        trim_txt += f"-to {end_time} "

    # Check if CUDA hardware acceleration is available
    hwaccel_available = False
    try:
        cmd = "ffmpeg -hide_banner -hwaccels | grep cuda"
        encoders = subprocess.check_output(cmd, shell=True, text=True)
        if "cuda" in encoders:
            hwaccel_available = True
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(
            "WARNING: cuda hwaccel not available, frame extraction will be slow. "
            "Install NVIDIA drivers and ffmpeg with cuda support for better speeds."
        )

    hwaccel = " -hwaccel cuda" if hwaccel_available else ""

    # Construct the final command string
    ffmpeg_cmd = (
        f"ffmpeg{hwaccel} {trim_txt}-i {video_path} "
        f'-vf "select=not(mod(n\\,{sampling_step}))" -vsync vfr '
        f"{output_dir}/%04d.jpg"
    )

    # Ensure the output directory exists and run the command
    os.makedirs(output_dir, exist_ok=True)
    subprocess.run(ffmpeg_cmd, shell=True)