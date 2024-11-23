from dataclasses import dataclass


from calib_board.core.checkerboard import CheckerboardMotion
from calib_board.core.checkerboardGeometry import CheckerboardGeometry


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