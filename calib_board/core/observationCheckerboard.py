"""
This module defines the ObservationCheckerboard class, which represents a
single observation of a checkerboard by a camera.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


class ObservationCheckerboard:
    """
    Represents a single observation of a checkerboard's corners in an image.

    This class stores the 2D coordinates of the detected corners and tracks
    their conformity, both for the individual points and the observation as a whole.
    The attributes use a leading underscore to indicate they are part of the
    internal state of the calibration package and may be manipulated by other
    core components like the `ExternalCalibrator`.

    Attributes:
        _2d (Optional[np.ndarray]): An (N, 2) array of detected 2D corner
                                    coordinates.
        _is_conform (bool): A flag indicating if the observation as a whole is
                            considered reliable and should be used in calibration.
        _conformity_mask (np.ndarray): A boolean array of shape (N,) where True
                                       indicates that the corresponding 2D point is
                                       an inlier based on reprojection error.
    """

    _2d: Optional[np.ndarray]
    _is_conform: bool
    _conformity_mask: np.ndarray

    def __init__(
        self,
        points_2d: Optional[np.ndarray] = None,
        is_conform: bool = True,
    ) -> None:
        """
        Initializes an ObservationCheckerboard instance.

        Args:
            points_2d: An (N, 2) numpy array of detected 2D corner coordinates.
                       If None, the observation is considered invalid from the start.
            is_conform: A boolean flag indicating if the entire observation is
                        initially considered conformant.
        """
        self._2d = points_2d
        self._is_conform = is_conform

        # If no points are provided, the observation is non-conform and has an
        # empty conformity mask.
        if self._2d is None:
            self._conformity_mask = np.array([], dtype=bool)
            self._is_conform = False
            return

        # By default, all initially provided points are considered conform.
        self._conformity_mask = np.ones(self._2d.shape[0], dtype=bool)

    def get_num_conform_corners(self) -> int:
        """
        Returns the number of corner points considered to be inliers.

        Returns:
            The integer count of conform corners based on the conformity mask.
        """
        return int(self._conformity_mask.sum())

    def get_2d_conform(self) -> np.ndarray:
        """
        Returns the 2D coordinates of all conform (inlier) corner points.

        Returns:
            An (M, 2) numpy array of the conform 2D points, where M is the
            number of conform corners. Returns an empty (0, 2) array if there
            are no points or no conform points.
        """
        if self._2d is None:
            return np.empty((0, 2), dtype=float)
        return self._2d[self._conformity_mask, :]

    def get_2d_nonconform(self) -> np.ndarray:
        """
        Returns the 2D coordinates of all non-conform (outlier) corner points.

        Returns:
            An (K, 2) numpy array of the non-conform 2D points, where K is the
            number of non-conform corners. Returns an empty (0, 2) array if
            there are no points or no non-conform points.
        """
        if self._2d is None:
            return np.empty((0, 2), dtype=float)
        return self._2d[~self._conformity_mask, :]