import dataclasses
from dataclasses import dataclass
from pathlib import Path

import cv2
from calib_board.core.checkerboard import CheckerboardMotion
from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calib_commons.utils.detect_board import BoardType


@dataclass
class ExternalCalibratorConfig:
    """Configuration defining the parameters use in the external calibrator."""
    checkerboard_geometry: CheckerboardGeometry
    checkerboard_motion: CheckerboardMotion
    min_track_length: int = 2
    reprojection_error_threshold: float = 1
    min_number_of_valid_observed_points_per_checkerboard_view: int = 10
    camera_score_threshold: float = 200
    ba_least_square_ftol: float = 1e-6
    least_squares_verbose: int = 1
    verbose: int = 1


@dataclass
class Config:
    # Input folder containing time-synchronized images or videos
    input_folder: str
    # Folder containing camera intrinsics
    intrinsics_folder: str
    # Output folder
    out_folder_calib: str = "results"

    # PRE-PROCESSING PARAMETERS

    # Sample stride for time-synchronized videos
    sampling_step: int = 5
    # Start time for window to sample images from time-synchronized videos (in hh:mm:ss.ms)
    start_time_window: str | None = "00:00:00.000"
    # End time for window to sample images from time-synchronized videos (in hh:mm:ss.ms, default end of video)
    end_time_window: str | None = None

    # Geometry of the checkerboard used for external calibration
    checkerboard_geometry: CheckerboardGeometry = dataclasses.field(
        default_factory=lambda: CheckerboardGeometry(
            rows=4, columns=6, square_size=0.165  # internal rows  # internal columns
        )
    )  # [m]
    # Board type used for external calibration
    board_type: BoardType = BoardType.CHARUCO
    # Charuco marker size (only for board_type=BoardType.CHARUCO)
    charuco_marker_size: float = 0.123
    # Charuco dictionary (only for board_type=BoardType.CHARUCO)
    charuco_dictionary: int = cv2.aruco.DICT_4X4_100

    # Show detection images
    show_detection_images = False
    # Save detection images
    save_detection_images = False

    # CALIBRATION PARAMETERS

    # External calibrator configuration
    external_calibrator_config: ExternalCalibratorConfig = dataclasses.field(
        default_factory=lambda: ExternalCalibratorConfig(
            checkerboard_motion=CheckerboardMotion.FREE,  # CheckerboardMotion.PLANAR or CheckerboardMotion.FREE
            min_track_length=2,  # min number of camera per object (=3D) point
            checkerboard_geometry=CheckerboardGeometry(rows=4, columns=6, square_size=0.165),  # will be overwritten by checkerboard_geometry from above
            reprojection_error_threshold=1,  # [pix]
            min_number_of_valid_observed_points_per_checkerboard_view=10,
            ba_least_square_ftol=1e-6,  # Non linear Least-Squares
            least_squares_verbose=0,  # 0: silent, 1: report only final results, 2: report every iteration
            camera_score_threshold=200,
            verbose=1,  # 0: only final report, 1: only camera name when added, 2: full verbose
        )
    )

    # DEBUG parameters

    # Show visualization
    show_viz: bool = True
    # Save visualization
    save_viz: bool = True
    # Save evaluation metrics to json
    save_eval_metrics_to_json: bool = True
    # Save COLMAP reconstruction
    save_colmap_reconstruction: bool = False
