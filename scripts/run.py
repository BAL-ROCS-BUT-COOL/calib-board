import logging
import shutil
from pathlib import Path
from typing import Any, Dict

import cv2
import matplotlib.pyplot as plt
import numpy as np
import tyro

# Assuming these are third-party or project-specific libraries
from calib_board.core.config import Config
from calib_board.core.correspondences import (
    filter_correspondences_with_non_nan_points,
    filter_correspondences_with_track_length,
)
from calib_board.core.externalCalibrator import ExternalCalibrator, WorldFrame
from calib_board.utils import visualization
from calib_board.utils.convert_to_generic import (
    convert_checker_scene_to_generic_scene,
    convert_to_generic_correspondences,
)
from calib_board.utils.utils import (
    convert_correspondences_array_to_checker_correspondences,
    extract_frames_from_video,
)
from calib_commons.data.data_pickle import load_from_pickle, save_to_pickle
from calib_commons.data.load_calib import construct_cameras_intrinsics
from calib_commons.eval_generic_scene import eval_generic_scene
from calib_commons.scene import Scene, SceneType
from calib_commons.utils.detect_board import BoardType, detect_board_corners
from calib_commons.viz import visualization as generic_vizualization

# Configure logging for better feedback
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def prepare_image_folder(cfg: Config) -> Path:
    """
    Checks if the input is videos. If so, extracts frames.
    Returns the path to the folder containing images for processing.

    Args:
        cfg: The main configuration object.

    Returns:
        The path to the folder with images (original or sampled).
    """
    input_path = Path(cfg.input_folder)
    files = [f for f in input_path.iterdir() if f.is_file()]
    is_video = files and files[0].suffix.lower() in [".mp4", ".avi", ".mov"]

    if not is_video:
        logging.info(f"Processing images directly from: {input_path}")
        return input_path

    sampled_images_folder = input_path / "sampled_images"

    if cfg.force_rerun and sampled_images_folder.exists():
        logging.info(
            f"'force_rerun' is True. Deleting existing folder: \
                {sampled_images_folder}"
        )
        shutil.rmtree(sampled_images_folder)

    if sampled_images_folder.exists() and any(sampled_images_folder.iterdir()):
        logging.info(f"Using cached frames from: {sampled_images_folder}")
        return sampled_images_folder

    logging.info("Input is video. Sampling frames...")
    sampled_images_folder.mkdir(parents=True, exist_ok=True)
    video_files = [
        f for f in input_path.iterdir()
        if f.is_file()
        and f.suffix.lower() in [".mp4", ".avi", ".mov"]
    ]

    for video_file in video_files:
        logging.info(f"Extracting frames from {video_file.name}...")
        subfolder = sampled_images_folder / video_file.stem
        extract_frames_from_video(
            str(video_file),
            str(subfolder),
            cfg.sampling_step,
            cfg.start_time_window,
            cfg.end_time_window
        )

    logging.info(f"Frames saved to: {sampled_images_folder}")
    return sampled_images_folder


def detect_or_load_correspondences(
    cfg: Config, image_folder: Path
) -> Dict[str, Any]:
    """
    Detects board corners or loads them from a cached file if available.

    Args:
        cfg: The main configuration object.
        image_folder: Path to the folder containing images.

    Returns:
        A dictionary of detected correspondences.
    """
    cache_file = Path(cfg.out_folder_calib) / "correspondences_detected.pkl"

    if not cfg.force_rerun and cache_file.exists():
        logging.info(f"Loading cached correspondences from {cache_file}")
        return load_from_pickle(cache_file)

    logging.info("Detecting board corners...")
    charuco_detector = None
    if cfg.board_type == BoardType.CHARUCO:
        board = cv2.aruco.CharucoBoard(
            (cfg.checkerboard_geometry.columns + 1,
             cfg.checkerboard_geometry.rows + 1),
            cfg.checkerboard_geometry.square_size,
            cfg.charuco_marker_size,
            cv2.aruco.getPredefinedDictionary(cfg.charuco_dictionary),
        )
        charuco_detector = cv2.aruco.CharucoDetector(board)

    correspondences_nparray = detect_board_corners(
        images_parent_folder=image_folder,
        board_type=cfg.board_type,
        charuco_detector=charuco_detector,
        columns=cfg.checkerboard_geometry.columns,
        rows=cfg.checkerboard_geometry.rows,
        intrinsics_path=Path(cfg.intrinsics_path),
        undistort=True,
        display=cfg.show_detection_images,
        save_images_with_overlayed_detected_corners=cfg.save_detection_images,
    )
    correspondences = convert_correspondences_array_to_checker_correspondences(
        correspondences_nparray
    )

    logging.info(f"Saving detected correspondences to {cache_file}")
    save_to_pickle(cache_file, correspondences)

    return correspondences


def run_calibration(
    correspondences: Dict[str, Any], intrinsics: Dict[str, Any], cfg: Config
) -> tuple[Scene, Dict[str, Any]]:
    """
    Filters correspondences and runs the external calibration process.

    Args:
        correspondences: The detected corner correspondences.
        intrinsics: Camera intrinsics.
        cfg: The main configuration object.

    Returns:
        A tuple containing the estimated scene and the filtered
        correspondences.
    """
    logging.info("Filtering correspondences...")
    filtered_corr = filter_correspondences_with_non_nan_points(
        correspondences,
        cfg.external_calibrator_config.min_number_of_valid_observed_points_per_checkerboard_view
    )
    filtered_corr = filter_correspondences_with_track_length(
        filtered_corr, cfg.external_calibrator_config.min_track_length
    )

    logging.info("Starting external calibration...")
    external_calibrator = ExternalCalibrator(
        correspondences=filtered_corr,
        intrinsics=intrinsics,
        config=cfg.external_calibrator_config,
    )
    external_calibrator.calibrate()

    logging.info("Calibration finished. Retrieving scene...")
    checkerboard_scene_estimate = external_calibrator.get_scene(
        world_frame=WorldFrame.CAM_FIRST_CHOOSEN
    )
    checkerboard_correspondences = external_calibrator.correspondences

    checkerboard_scene_estimate.print_cameras_poses()
    return checkerboard_scene_estimate, checkerboard_correspondences


def process_and_save_results(
    checkerboard_scene_estimate: Scene,
    checkerboard_correspondences: Dict[str, Any],
    cfg: Config
) -> tuple[Scene, Dict[str, Any]]:
    """
    Converts results to a generic format, saves them, and evaluates the scene.

    Args:
        checkerboard_scene_estimate: The calibrated scene from the calibrator.
        checkerboard_correspondences: The correspondences used for calibration.
        cfg: The main configuration object.

    Returns:
        A tuple of the generic scene and generic observations.
    """
    out_dir = Path(cfg.out_folder_calib)
    logging.info("Converting results to generic scene format.")
    generic_scene = convert_checker_scene_to_generic_scene(
        checkerboard_scene_estimate,
        scene_type=SceneType.ESTIMATE
    )
    generic_obsv = convert_to_generic_correspondences(
        checkerboard_correspondences
    )

    # Save files
    cam_poses_file = out_dir / "camera_poses.json"
    generic_scene.save_cameras_poses_to_json(cam_poses_file)
    logging.info(f"Camera poses saved to: {cam_poses_file}")

    scene_estimate_file = out_dir / "scene_estimate.pkl"
    save_to_pickle(scene_estimate_file, generic_scene)
    logging.info(f"Scene estimate saved to: {scene_estimate_file}")

    correspondences_file = out_dir / "correspondences.pkl"
    save_to_pickle(correspondences_file, generic_obsv)
    logging.info(f"Correspondences saved to: {correspondences_file}")

    # Evaluate scene
    logging.info("Evaluating scene reprojection errors...")
    metrics_path = out_dir / "metrics.json" if cfg.save_eval_metrics_to_json else None
    eval_generic_scene(
        generic_scene,
        generic_obsv,
        camera_groups=None,
        save_to_json=cfg.save_eval_metrics_to_json,
        output_path=metrics_path,
        print_=True,
    )

    if cfg.save_colmap_reconstruction:
        colmap_path = out_dir / "colmap"
        logging.info(f"Saving COLMAP reconstruction to: {colmap_path}")
        generic_scene.save_colmap(colmap_path)

    return generic_scene, generic_obsv


def generate_visualizations(
    checkerboard_scene: Scene,
    checkerboard_corr: Dict,
    generic_scene: Scene,
    generic_obsv: Dict,
    cfg: Config,
):
    """
    Generates and saves/shows all requested visualizations.

    Args:
        checkerboard_scene: The original checkerboard scene estimate.
        checkerboard_corr: The checkerboard correspondences.
        generic_scene: The scene in generic format.
        generic_obsv: The observations in generic format.
        cfg: The main configuration object.
    """
    if not (cfg.show_viz or cfg.save_viz):
        return

    out_dir = Path(cfg.out_folder_calib)
    logging.info("Generating visualizations...")

    # 3D Scene Visualization
    save_path = out_dir / "scene_3d.png"
    visualization.visualize_scenes(
        [checkerboard_scene],
        show_ids=False,
        show_fig=cfg.show_viz,
        save_fig=cfg.save_viz,
        save_path=save_path
    )
    if cfg.save_viz:
        logging.info(f"3D scene visualization saved to {save_path}")

    # 2D Detections Visualization
    save_path = out_dir / "detections_2d.png"
    visualization.visualize_2d(
        checkerboard_scene,
        checkerboard_corr,
        which="both",
        show_ids=False,
        show_fig=cfg.show_viz,
        save_fig=cfg.save_viz,
        save_path=save_path
    )
    if cfg.save_viz:
        logging.info(f"2D detections visualization saved to {save_path}")

    # 2D Reprojection Errors Visualization
    save_path = out_dir / "reprojection_errors_2d.png"
    generic_vizualization.plot_reprojection_errors(
        scene_estimate=generic_scene,
        observations=generic_obsv,
        show_fig=cfg.show_viz,
        save_fig=cfg.save_viz,
        save_path=save_path,
    )
    if cfg.save_viz:
        logging.info(f"2D reprojection error vis saved to {save_path}")

    if cfg.show_viz:
        logging.info("Displaying visualization windows...")
        plt.show()


def main(cfg: Config):
    """
    Main function to run the entire calibration pipeline.
    """
    # --- 1. Setup and Pre-processing ---
    # Ensure output directory exists
    Path(cfg.out_folder_calib).mkdir(parents=True, exist_ok=True)

    # Use original input folder or extract frames from videos
    image_folder = prepare_image_folder(cfg)

    # --- 2. Corner Detection ---
    # This step is cached to avoid re-running on subsequent executions
    correspondences = detect_or_load_correspondences(cfg, image_folder)

    # --- 3. External Calibration ---
    intrinsics = construct_cameras_intrinsics(
        image_folder,
        Path(cfg.intrinsics_path)
    )

    # Update config with correct checkerboard geometry for the calibrator
    cfg.external_calibrator_config.checkerboard_geometry = cfg.checkerboard_geometry

    checkerboard_scene, checkerboard_corr = run_calibration(
        correspondences,
        intrinsics,
        cfg
    )

    # --- 4. Save Results and Evaluate ---
    generic_scene, generic_obsv = process_and_save_results(
        checkerboard_scene,
        checkerboard_corr,
        cfg
    )

    # --- 5. Visualization ---
    generate_visualizations(
        checkerboard_scene, 
        checkerboard_corr, 
        generic_scene, 
        generic_obsv, 
        cfg
    )

    logging.info("Calibration pipeline finished successfully.")


if __name__ == "__main__":
    # --- Configuration ---
    # Set a fixed random seed for reproducibility
    np.random.seed(1)

    # Parse command-line arguments using tyro
    cfg = tyro.cli(Config)

    # --- Execution ---
    main(cfg)
