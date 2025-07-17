"""
This module defines configuration classes for the camera calibration pipeline,
encapsulating all necessary parameters for processing, calibration, and debugging.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import cv2
from calib_board.core.checkerboard import CheckerboardMotion
from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calib_commons.utils.detect_board import BoardType


@dataclass
class ExternalCalibratorConfig:
    """
    Configuration defining the parameters for the external calibrator.

    Attributes:
        checkerboard_geometry: The geometry of the calibration checkerboard.
        checkerboard_motion: The motion model for the checkerboard (PLANAR or FREE).
        min_track_length: The minimum number of camera views required for a 3D
                          point to be considered valid.
        reprojection_error_threshold: The maximum reprojection error (in pixels)
                                      for an observation to be considered an inlier.
        min_number_of_valid_observed_points_per_checkerboard_view: The minimum
            number of detected corners required for a checkerboard view to be used.
        camera_score_threshold: A threshold for a camera's score to be
                                considered well-calibrated.
        ba_least_square_ftol: The function tolerance for the bundle adjustment's
                              least-squares optimizer.
        least_squares_verbose: Verbosity level for the least-squares solver.
                               0=silent, 1=final report, 2=report each iteration.
        fix_first: Whether to fix the position of the first camera
        verbose: General verbosity level for the calibrator.
    """

    checkerboard_geometry: CheckerboardGeometry
    checkerboard_motion: CheckerboardMotion
    min_track_length: int = 2
    reprojection_error_threshold: float = 1.0
    min_number_of_valid_observed_points_per_checkerboard_view: int = 10
    camera_score_threshold: float = 200.0
    ba_least_square_ftol: float = 1e-6
    least_squares_verbose: int = 1
    free_first: bool = False
    verbose: int = 1


@dataclass
class Config:
    """
    Main configuration class for the entire calibration pipeline.

    Attributes:
        input_folder: Path to the folder containing time-synchronized images or videos.
        intrinsics_folder: Path to the folder containing camera intrinsics files.
        out_folder_calib: Name of the output folder for calibration results.
        sampling_step: Stride for sampling frames from videos.
        start_time_window: Start time for sampling (e.g., "00:00:00.000").
                           If None, starts from the beginning.
        end_time_window: End time for sampling. If None, samples until the end.
        checkerboard_geometry: The primary definition of the checkerboard's geometry.
        board_type: The type of calibration board (e.g., CHARUCO).
        charuco_marker_size: The size of the ArUco markers in a ChArUco board [m].
        charuco_dictionary: The dictionary used for the ChArUco board.
        show_detection_images: If True, display detection images during processing.
        save_detection_images: If True, save detection images to the output folder.
        external_calibrator_config: A nested configuration object for the
                                    external calibration algorithm.
        show_viz: If True, display 3D visualizations of the results.
        save_viz: If True, save 3D visualizations to the output folder.
        save_eval_metrics_to_json: If True, save evaluation metrics to a JSON file.
        save_colmap_reconstruction: If True, save the reconstruction in a
                                    COLMAP-compatible format.
    """

    # --- INPUT/OUTPUT PARAMETERS ---
    input_folder: str
    intrinsics_folder: str
    out_folder_calib: str = "results"

    # --- PRE-PROCESSING PARAMETERS ---
    sampling_step: int = 5
    start_time_window: Optional[str] = "00:00:00.000"
    end_time_window: Optional[str] = None
    force_rerun: bool = False

    # --- BOARD GEOMETRY PARAMETERS ---
    checkerboard_geometry: CheckerboardGeometry = field(
        default_factory=lambda: CheckerboardGeometry(
            rows=4, columns=6, square_size=0.165
        )
    )
    board_type: BoardType = BoardType.CHARUCO
    charuco_marker_size: float = 0.123
    charuco_dictionary: int = cv2.aruco.DICT_4X4_100

    # --- DETECTION PARAMETERS ---
    show_detection_images: bool = False
    save_detection_images: bool = False

    # --- CALIBRATION PARAMETERS ---
    external_calibrator_config: ExternalCalibratorConfig = field(
        default_factory=lambda: ExternalCalibratorConfig(
            checkerboard_motion=CheckerboardMotion.FREE,
            min_track_length=2,
            checkerboard_geometry=CheckerboardGeometry(
                rows=4, columns=6, square_size=0.165
            ),
            reprojection_error_threshold=1.0,
            min_number_of_valid_observed_points_per_checkerboard_view=10,
            ba_least_square_ftol=1e-6,
            least_squares_verbose=2,
            camera_score_threshold=200.0,
            free_first=True,
            verbose=2,
        )
    )

    # --- DEBUG AND VISUALIZATION PARAMETERS ---
    show_viz: bool = False
    save_viz: bool = True
    save_eval_metrics_to_json: bool = True
    save_colmap_reconstruction: bool = False


    def __post_init__(self) -> None:
        """
        Ensures consistency between configuration parameters after initialization.
        """
        # The external calibrator should use the same checkerboard geometry
        # defined at the top level of this configuration. This overwrites the
        # default value in the ExternalCalibratorConfig factory.
        self.external_calibrator_config.checkerboard_geometry = (
            self.checkerboard_geometry
        )