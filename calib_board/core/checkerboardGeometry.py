"""
This module defines the CheckerboardGeometry class, which represents the
intrinsic geometric properties of a calibration checkerboard.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class CheckerboardGeometry:
    """
    Represents the geometric properties of a checkerboard.

    This class computes and stores the 3D coordinates of the checkerboard
    corners in its own local coordinate frame. The origin (0,0,0) is at one
    corner of the grid.

    Attributes:
        rows: Number of internal corner rows.
        columns: Number of internal corner columns.
        square_size: The side length of a single square in meters.
        _3d: (N, 3) array of internal corner coordinates in the board's frame.
        _3d_all: (M, 3) array of all corner coordinates (internal + external
                 border) in the board's frame.
        _3d_augmented: (4, N) array of augmented internal corner coordinates for
                       homogeneous transformations.
        _3d_all_augmented: (4, M) array of augmented all corner coordinates.
    """

    rows: int
    columns: int
    square_size: float

    # Attributes computed after initialization
    _3d: np.ndarray = field(init=False, repr=False)
    _3d_all: np.ndarray = field(init=False, repr=False)
    _3d_augmented: np.ndarray = field(init=False, repr=False)
    _3d_all_augmented: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """
        Computes and caches the corner coordinates after the object is created.
        """
        self._3d = self._generate_corners_3d()
        self._3d_all = self._generate_corners_3d_all()
        self._3d_augmented = self._generate_corners_3d_augmented()
        self._3d_all_augmented = self._generate_corners_3d_all_augmented()

    def _generate_corners_2d(self) -> np.ndarray:
        """Generates 2D coordinates of the internal corners."""
        i_indices, j_indices = np.indices((self.rows, self.columns))
        x_coords = i_indices.ravel() * self.square_size
        y_coords = j_indices.ravel() * self.square_size
        return np.stack((x_coords, y_coords), axis=-1)

    def _generate_corners_2d_all(self) -> np.ndarray:
        """Generates 2D coordinates of all corners, including the border."""
        i_indices, j_indices = np.indices((self.rows + 2, self.columns + 2))
        x_coords = (i_indices.ravel() - 1) * self.square_size
        y_coords = (j_indices.ravel() - 1) * self.square_size
        return np.stack((x_coords, y_coords), axis=-1)

    def _generate_corners_3d(self) -> np.ndarray:
        """Generates 3D coordinates of the internal corners (Z=0)."""
        corners_2d = self._generate_corners_2d()
        return np.hstack((corners_2d, np.zeros((corners_2d.shape[0], 1))))

    def _generate_corners_3d_all(self) -> np.ndarray:
        """Generates 3D coordinates of all corners, including the border (Z=0)."""
        corners_2d = self._generate_corners_2d_all()
        return np.hstack((corners_2d, np.zeros((corners_2d.shape[0], 1))))

    def _generate_corners_3d_augmented(self) -> np.ndarray:
        """Generates augmented 3D coordinates of internal corners."""
        augmented_points = np.ones((4, self._3d.shape[0]))
        augmented_points[:3, :] = self._3d.T
        return augmented_points

    def _generate_corners_3d_all_augmented(self) -> np.ndarray:
        """Generates augmented 3D coordinates of all corners."""
        augmented_points = np.ones((4, self._3d_all.shape[0]))
        augmented_points[:3, :] = self._3d_all.T
        return augmented_points

    def get_3d_points_in_world(self, pose: np.ndarray) -> np.ndarray:
        """
        Transforms the internal 3D corner points to the world frame.

        Args:
            pose: A 4x4 homogeneous transformation matrix representing the
                  checkerboard's pose in the world frame.

        Returns:
            A numpy array of shape (N, 3) containing the 3D world coordinates
            of the internal corners.
        """
        points_in_world_aug = pose @ self._3d_augmented
        return points_in_world_aug[:3, :].T

    def get_internal_and_external_3d_points_in_world(
        self, pose: np.ndarray
    ) -> np.ndarray:
        """
        Transforms all 3D corner points (internal and external) to the world frame.

        Args:
            pose: A 4x4 homogeneous transformation matrix representing the
                  checkerboard's pose in the world frame.

        Returns:
            A numpy array of shape (M, 3) containing the 3D world coordinates
            of all corners.
        """
        points_in_world_aug = pose @ self._3d_all_augmented
        return points_in_world_aug[:3, :].T

    def get_center_checkerboard(self) -> np.ndarray:
        """
        Calculates the geometric center of the checkerboard's internal corners.

        Returns:
            A numpy array of shape (2,) with the [x, y] coordinates of the center.
        """
        # Note: This calculates the center of the bounding box of the corners
        # in the board's local frame.
        center_x = ((self.columns - 1) * self.square_size) / 2
        center_y = ((self.rows - 1) * self.square_size) / 2
        return np.array([center_x, center_y])

    def get_num_corners(self) -> int:
        """
        Returns the total number of internal corners.

        Returns:
            The integer count of internal corners.
        """
        return self.rows * self.columns


# Example usage
if __name__ == "__main__":
    # Create an instance of the checkerboard geometry
    # (e.g., 9x14 internal corners, 1-unit square size)
    checkerboard_geometry = CheckerboardGeometry(rows=9, columns=14, square_size=1.0)

    print("Number of internal corners:", checkerboard_geometry.get_num_corners())
    print("\n3D Internal Corners (in board frame):")
    print(checkerboard_geometry._3d)

    print("\n3D All Corners (in board frame):")
    print(checkerboard_geometry._3d_all)

    print("\nCenter of Checkerboard (internal corners):")
    print(checkerboard_geometry.get_center_checkerboard())