import os
import subprocess 

import numpy as np


from calib_board.core.observationCheckerboard import ObservationCheckerboard

def is_chessboard_visible(camera_pose: np.ndarray, chessboard_pose: np.ndarray) -> bool:
    """
    Determine if the chessboard is visible from the camera.

    Parameters:
    camera_pose (np.ndarray): 4x4 transformation matrix representing the camera pose.
    chessboard_pose (np.ndarray): 4x4 transformation matrix representing the chessboard pose.

    Returns:
    bool: True if the chessboard is facing the camera, False otherwise.
    """

    # Check the input matrix dimensions
    assert camera_pose.shape == (4, 4), "camera_pose must be a 4x4 matrix"
    assert chessboard_pose.shape == (4, 4), "chessboard_pose must be a 4x4 matrix"

    # Step 1: Compute the relative transformation matrix from chessboard to camera
    camera_pose_inv = np.linalg.inv(camera_pose)
    chessboard_in_camera = camera_pose_inv @ chessboard_pose

    # Step 2: Extract the rotation matrix of the chessboard relative to the camera
    rotation_matrix = chessboard_in_camera[:3, :3]

    # Step 3: Define the chessboard normal vector in its local frame
    chessboard_normal_local = np.array([0, 0, 1])  # z-axis in chessboard frame

    # Step 4: Transform the chessboard normal vector to the camera frame
    chessboard_normal_camera = rotation_matrix @ chessboard_normal_local

    # Step 5: Camera's viewing direction in its own frame (negative z-axis)
    camera_viewing_direction = np.array([0, 0, -1])

    # Step 6: Compute the dot product between chessboard normal and camera viewing direction
    dot_product = np.dot(chessboard_normal_camera, camera_viewing_direction)

    # Step 7: Check if the chessboard is facing the camera (dot_product > 0 means visible)
    return dot_product > 0


def convert_correspondences_array_to_checker_correspondences(correspondences): 
    correspondences_checker = {}
    for cam in correspondences:
        correspondences_checker[cam] = {}
        for checker_id, _2d in correspondences[cam].items():
                correspondences_checker[cam][checker_id] = ObservationCheckerboard(_2d)
    return correspondences_checker


def extract_frames_from_video(video_path: str, output_dir: str, sampling_step: int, start_time: str = None, end_time: str = None):
    trim_txt = " "
    if start_time is not None:
        trim_txt += f"-ss {start_time} "
    if end_time is not None:
        trim_txt += f"-to {end_time} "

    # Check if CUDA acceleration is available
     # Check if nvenc is available for speed up
    nvenc_available = False
    try:
        cmd = "ffmpeg -hide_banner -hwaccels | grep cuda"
        encoders = subprocess.check_output(cmd, shell=True).decode("utf-8")
        if "cuda" not in encoders:
            raise subprocess.CalledProcessError(1, cmd)
        else:
            nvenc_available = True
    except subprocess.CalledProcessError:
        print("WARNING: cuda hwaccel not available, frame extraction will be slow. Install NVIDIA drivers and ffmpeg with cuda support for better speeds.")

    hwaccel = " -hwaccel cuda" if nvenc_available else ""
    # nvenc = " -c:v h264_nvenc" if nvenc_available else ""

    ffmpeg_cmd = f"ffmpeg{hwaccel}{trim_txt}-i {video_path} -vf \"select=not(mod(n\,{sampling_step}))\" -vsync vfr {output_dir}/%04d.jpg"
    os.makedirs(output_dir, exist_ok=True)
    subprocess.run(ffmpeg_cmd, shell=True)