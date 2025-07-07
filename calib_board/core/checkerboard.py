"""
This module defines the Checkerboard and its associated motion types, which are
fundamental components for camera calibration tasks.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

# Local application/library specific imports
from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calib_commons.types import idtype
from calib_commons.utils.se3 import SE3
from calib_commons.viz.visualization_tools import get_color_from_id, plot_frame


class CheckerboardMotion(Enum):
    """
    Enumeration for the type of motion a checkerboard can undergo.

    Attributes:
        FREE: The checkerboard can move freely in 6 degrees of freedom (DoF).
        PLANAR: The checkerboard is constrained to move on a 2D plane (3 DoF).
    """

    FREE = "Free"
    PLANAR = "Planar"


class Checkerboard:
    """
    Represents a physical checkerboard in 3D space.

    This class stores the checkerboard's identity, its 3D pose, and its
    geometric properties (e.g., number of squares, square size).

    Attributes:
        id (idtype): A unique identifier for the checkerboard.
        pose (SE3): The 3D pose of the checkerboard in the world frame,
                    represented as an SE3 object.
        checkerboard_geometry (CheckerboardGeometry): An object containing the
                                                     intrinsic geometric
                                                     properties of the board.
    """

    id: idtype
    pose: SE3
    checkerboard_geometry: CheckerboardGeometry

    def __init__(
        self,
        id: idtype,
        pose: SE3,
        checkerboard_geometry: CheckerboardGeometry,
    ) -> None:
        """
        Initializes a Checkerboard instance.

        Args:
            id: A unique identifier for the checkerboard.
            pose: The 3D pose (SE3 object) of the checkerboard in the world frame.
            checkerboard_geometry: An object defining the board's geometry.
        """
        if not isinstance(pose, SE3):
            raise ValueError("pose must be an instance of SE3")

        self.id = id
        self.pose = pose
        self.checkerboard_geometry = checkerboard_geometry

    def plot(self, name: Optional[str] = None, ax: Optional[plt.Axes] = None) -> None:
        """
        Plots the checkerboard's 3D corners and its coordinate frame.

        Args:
            name: An optional name to display for the checkerboard's frame label.
                  If None, a default label is generated using the board's ID.
            ax: The matplotlib axes on which to plot. If None, the current
                axes are retrieved using `plt.gca()`.
        """
        if ax is None:
            ax = plt.gca()

        corners_3d = self.get_3d_points_in_world()
        ax.scatter(
            corners_3d[:, 0],
            corners_3d[:, 1],
            corners_3d[:, 2],
            color=get_color_from_id(self.id),
            s=2,
        )

        if name is None:
            name = r"$\{B_{" + str(self.id) + r"}\}$"

        plot_frame(self.pose.mat, name, axis_length=0.15, ax=ax)

    def get_3d_points_in_world(self) -> np.ndarray:
        """
        Calculates the 3D coordinates of the checkerboard corners in the world frame.

        Returns:
            A numpy array of shape (N, 3) containing the 3D coordinates of the
            checkerboard corners.
        """
        return self.checkerboard_geometry.get_3d_points_in_world(self.pose.mat)

    def get_internal_and_external_3d_points_in_world(
        self,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the 3D coordinates of both internal and external checkerboard
        corners in the world frame.

        Returns:
            A tuple containing two numpy arrays:
            - The internal corners' 3D coordinates.
            - The external corners' 3D coordinates.
        """
        return self.checkerboard_geometry.get_internal_and_external_3d_points_in_world(
            self.pose.mat
        )