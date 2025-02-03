import matplotlib.pyplot as plt
import numpy as np 
from pathlib import Path
import cv2

from calib_commons.data.load_calib import construct_cameras_intrinsics
from calib_commons.data.data_pickle import save_to_pickle, load_from_pickle
from calib_commons.eval_generic_scene import eval_generic_scene
from calib_commons.scene import SceneType
from calib_commons.viz import visualization as generic_vizualization
from calib_commons.utils.detect_board import detect_board_corners, BoardType

from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calib_board.core.checkerboard import CheckerboardMotion
from calib_board.core.config import ExternalCalibratorConfig
from calib_board.core.externalCalibrator import ExternalCalibrator, WorldFrame
from calib_board.core.correspondences import filter_correspondences_with_track_length, filter_correspondences_with_non_nan_points
from calib_board.utils.convert_to_generic import convert_checker_scene_to_generic_scene, convert_to_generic_correspondences
from calib_board.utils.utils import convert_correspondences_array_to_checker_correspondences, extract_frames_from_video
from calib_board.utils import visualization
from calib_board.utils.config import Config

import tyro


# random seed
np.random.seed(1)

############################### USER INTERFACE ####################################

cfg = tyro.cli(Config)
cfg.input_folder = Path(cfg.input_folder)
cfg.intrinsics_folder = Path(cfg.intrinsics_folder)
cfg.out_folder_calib = Path(cfg.out_folder_calib)

###################### PRE-PROCESSING: SAMPLING FRAMES FROM VIDEO ###########################
files = [f for f in cfg.input_folder.iterdir() if f.is_file()]
is_video = files[0].suffix.lower() in [".mp4", ".avi", ".mov"] if len(files) > 0 else False


if is_video:
    # Sample frames from all videos and store into temporary folder
    sampled_images_folder = cfg.input_folder / "sampled_images"
    sampled_images_folder.mkdir(parents=True, exist_ok=True)

    for video_file in cfg.input_folder.iterdir():
        if not video_file.is_file() or video_file.suffix.lower() not in [".mp4", ".avi", ".mov"]:
            continue
        subfolder = sampled_images_folder / video_file.stem
        extract_frames_from_video(str(video_file), str(subfolder), cfg.sampling_step, cfg.start_time_window, cfg.end_time_window)

    cfg.input_folder = sampled_images_folder
###################### PRE-PROCESSING: CORNERS DETECTION ###########################

if cfg.board_type == BoardType.CHARUCO:
    charuco_detector = cv2.aruco.CharucoDetector(cv2.aruco.CharucoBoard((cfg.checkerboard_geometry.columns+1, cfg.checkerboard_geometry.rows+1), 
                                                                        cfg.checkerboard_geometry.square_size, 
                                                                        cfg.charuco_marker_size, 
                                                                        cv2.aruco.getPredefinedDictionary(cfg.charuco_dictionary)))
else: 
    charuco_detector = None

correspondences_nparray = detect_board_corners(images_parent_folder=cfg.input_folder,
                                        board_type=cfg.board_type,
                                        charuco_detector=charuco_detector,
                                        columns=cfg.checkerboard_geometry.columns, 
                                        rows=cfg.checkerboard_geometry.rows, 
                                        intrinsics_folder=cfg.intrinsics_folder, 
                                        undistort=True, 
                                        display=cfg.show_detection_images, 
                                        save_images_with_overlayed_detected_corners=cfg.save_detection_images)
correspondences = convert_correspondences_array_to_checker_correspondences(correspondences_nparray)

    # # save_to_pickle(out_folder_calib / "correspondences_detected.pkl", correspondences)
# # correspondences = load_from_pickle("results/correspondences_detected.pkl")


###################### EXTERNAL CALIBRATION ###########################

# keep only chessboard views with sufficient corners detected
correspondences = filter_correspondences_with_non_nan_points(correspondences, cfg.external_calibrator_config.min_number_of_valid_observed_points_per_checkerboard_view)
# keep only chessboard with sufficient track length
correspondences = filter_correspondences_with_track_length(correspondences, cfg.external_calibrator_config.min_track_length)

cfg.out_folder_calib.mkdir(parents=True, exist_ok=True)
intrinsics = construct_cameras_intrinsics(cfg.input_folder, cfg.intrinsics_folder)

# Calibrate
externalCalibrator = ExternalCalibrator(correspondences=correspondences, 
                                        intrinsics=intrinsics, 
                                        config=cfg.external_calibrator_config
                                        )
externalCalibrator.calibrate()
checkerboard_scene_estimate = externalCalibrator.get_scene(world_frame=WorldFrame.CAM_FIRST_CHOOSEN)
checkerboard_correspondences = externalCalibrator.correspondences
checkerboard_scene_estimate.print_cameras_poses()
generic_scene = convert_checker_scene_to_generic_scene(checkerboard_scene_estimate, scene_type=SceneType.ESTIMATE)
generic_obsv = convert_to_generic_correspondences(checkerboard_correspondences)

# Save files
generic_scene.save_cameras_poses_to_json(cfg.out_folder_calib / "camera_poses.json")
print("camera poses saved to", cfg.out_folder_calib / "camera_poses.json")
scene_estimate_file = cfg.out_folder_calib / "scene_estimate.pkl"
save_to_pickle(scene_estimate_file, generic_scene)
print("scene estimate saved to", scene_estimate_file)
correspondences_file = cfg.out_folder_calib / "correspondences.pkl"
save_to_pickle(correspondences_file, generic_obsv)
print("correspondences saved to", correspondences_file)
metrics = eval_generic_scene(generic_scene, generic_obsv, camera_groups=None, save_to_json=cfg.save_eval_metrics_to_json, output_path=cfg.out_folder_calib / "metrics.json", print_ = True)
print("")

if cfg.save_colmap_reconstruction:
    generic_scene.save_colmap(cfg.out_folder_calib / "colmap")

# Visualization
if cfg.show_viz or cfg.save_viz:
    dpi = 300
    save_path = cfg.out_folder_calib / "scene.png"
    visualization.visualize_scenes([checkerboard_scene_estimate], show_ids=False, show_fig=cfg.show_viz, save_fig=cfg.save_viz, save_path=save_path)
    if cfg.save_viz:
        print("scene visualization saved to", save_path)
    save_path = cfg.out_folder_calib / "2d.png"
    visualization.visualize_2d(checkerboard_scene_estimate, checkerboard_correspondences, which="both", subplots=True, show_ids=False, show_fig=cfg.show_viz, save_fig=cfg.save_viz, save_path=save_path)
    if cfg.save_viz:
        print("2d visualization saved to", save_path)
    save_path = cfg.out_folder_calib / "2d_errors.png"
    generic_vizualization.plot_reprojection_errors(scene_estimate=generic_scene, 
                                        observations=generic_obsv, 
                                        show_fig=cfg.show_viz,
                                        save_fig=cfg.save_viz, 
                                        save_path=save_path)
    if cfg.save_viz:
        print("2d errors visualization saved to", save_path)

    if cfg.show_viz:
        plt.show()
