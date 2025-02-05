from dataclasses import dataclass
import numpy as np

@dataclass
class CheckerboardGeometry:
    """
    A class to represent the geometry of a checkerboard used for calibration.
    Attributes
    ----------
    rows : int
        Number of rows of internal corners in the checkerboard.
    columns : int
        Number of columns of internal corners in the checkerboard.
    square_size : float
        Size of each square in the checkerboard (in meters).
    _3d : np.ndarray
        3D coordinates of the internal corners of the checkerboard.
    _3d_all : np.ndarray
        3D coordinates of all corners of the checkerboard including the border.
    _3d_augmented : np.ndarray
        Augmented 3D coordinates of the internal corners for homogeneous transformations.
    _3d_all_augmented : np.ndarray
        Augmented 3D coordinates of all corners for homogeneous transformations.
    Methods
    -------
    __generate_corners_2d() -> np.ndarray:
        Generates 2D coordinates of the internal corners of the checkerboard.
    __generate_corners_2d_all() -> np.ndarray:
        Generates 2D coordinates of all corners of the checkerboard including the border.
    __generate_corners_3d() -> np.ndarray:
        Generates 3D coordinates of the internal corners of the checkerboard.
    __generate_corners_3d_all() -> np.ndarray:
        Generates 3D coordinates of all corners of the checkerboard including the border.
    get_3d_points_in_world(pose: np.ndarray) -> np.ndarray:
        Transforms the internal 3D points to the world coordinate system using the given pose.
    get_internal_and_external_3d_points_in_world(pose: np.ndarray) -> np.ndarray:
        Transforms all 3D points (internal and external) to the world coordinate system using the given pose.
    generate_corners_3d_augmented() -> np.ndarray:
        Generates augmented 3D coordinates of the internal corners for homogeneous transformations.
    generate_corners_3d_all_augmented() -> np.ndarray:
        Generates augmented 3D coordinates of all corners for homogeneous transformations.
    get_center_checkerboard() -> np.ndarray:
        Calculates the geometric center of the checkerboard.
    get_num_corners() -> int:
        Returns the number of internal corners in the checkerboard.
    """
    rows: int
    columns: int
    square_size: float

    def __init__(self, 
                 rows: int,
                 columns: int, 
                 square_size: float):
        self.rows = rows
        self.columns = columns
        self.square_size = square_size

        self._3d = self.__generate_corners_3d()
        self._3d_all = self.__generate_corners_3d_all()
        self._3d_augmented = self.generate_corners_3d_augmented()
        self._3d_all_augmented = self.generate_corners_3d_all_augmented()

    def __generate_corners_2d(self) -> np.ndarray:
        i_indices, j_indices = np.indices((self.rows, self.columns))
        x_coords = i_indices * self.square_size
        y_coords = j_indices * self.square_size
        corners = np.stack((x_coords.ravel(), y_coords.ravel()), axis=-1)
        return corners

    def __generate_corners_2d_all(self) -> np.ndarray:
        i_indices, j_indices = np.indices((self.rows+2, self.columns+2))
        x_coords = i_indices * self.square_size - self.square_size
        y_coords = j_indices * self.square_size - self.square_size
        corners = np.stack((x_coords.ravel(), y_coords.ravel()), axis=-1)
        return corners

    def __generate_corners_3d(self) -> np.ndarray:
        # Generate 3D coordinates of the internal corners, iterating by rows first
        corners_2d = self.__generate_corners_2d()
        corners_3d = np.hstack((corners_2d, np.zeros((corners_2d.shape[0], 1))))
        return corners_3d
    
    def __generate_corners_3d_all(self) -> np.ndarray:
        # Generate 3D coordinates of the internal corners, iterating by rows first
        corners_2d = self.__generate_corners_2d_all()
        corners_3d = np.hstack((corners_2d, np.zeros((corners_2d.shape[0], 1))))
        return corners_3d
    
    def get_3d_points_in_world(self, pose: np.ndarray) -> np.ndarray: 
        points_in_world_aug = pose @ self._3d_augmented
        return points_in_world_aug[:3, :].T
    
    def get_internal_and_external_3d_points_in_world(self, pose: np.ndarray) -> np.ndarray: 
        points_in_world_aug = pose @ self._3d_all_augmented
        return points_in_world_aug[:3, :].T
    
    def generate_corners_3d_augmented(self) -> np.ndarray:
        # Generate 3D augmented coordinates of the internal corners, iterating by rows first
        corners_3d = self._3d
        augmented_points = np.ones((4, corners_3d.shape[0]))
        augmented_points[:3, :] = corners_3d.T
        return augmented_points

    def generate_corners_3d_all_augmented(self) -> np.ndarray:
        # Generate 3D augmented coordinates of the internal corners, iterating by rows first
        corners_3d = self._3d_all
        augmented_points = np.ones((4, corners_3d.shape[0]))
        augmented_points[:3, :] = corners_3d.T
        return augmented_points
    
    def get_center_checkerboard(self) -> np.ndarray:
        # Calculate the geometric center of the checkerboard
        center_x = (self.columns * self.square_size) / 2
        center_y = (self.rows * self.square_size) / 2
        return np.array([center_x, center_y])
    
    def get_num_corners(self) -> int: 
        return self.rows * self.columns

# Example usage
if __name__ == "__main__":
    checkerboard_geometry = CheckerboardGeometry(9, 14, 1)  # rows, columns, square size of 1 unit
    print("2D Corners by rows:")
    print(checkerboard_geometry.__generate_corners_2d())

    print("3D Corners by rows:")
    print(checkerboard_geometry.__generate_corners_3d())

    print("Center of Checkerboard:")
    print(checkerboard_geometry.get_center_checkerboard())
