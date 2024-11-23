import numpy as np 
import matplotlib.pyplot as plt
from enum import Enum 

from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calibCommons.viz.visualization_tools import plot_frame, get_color_from_id
from calibCommons.utils.se3 import SE3
from calibCommons.types import idtype

class CheckerboardMotion(Enum): 
    FREE = "Free"
    PLANAR = "Planar"

class Checkerboard: 
    def __init__(self,
                 id: idtype,
                 pose,
                 checkerboard_geometry: CheckerboardGeometry):
        
        self.id = id
        if not isinstance(pose, SE3): 
            raise ValueError("pose must be an instance of SE3")
        self.pose = pose
        self.checkerboard_geometry = checkerboard_geometry

    def plot(self, name=None, ax=None) -> None: 
        """
        Plots the checkerboard in 3D space.

        Parameters:
        name (str, optional): The name to display on the plot. Defaults to None.
        ax (matplotlib.axes._axes.Axes, optional): The axes on which to plot. Defaults to None.
        """
        if ax is None: 
            ax = plt.gca()
        corners_3d = self.get_3d_points_in_world()
        ax.scatter(corners_3d[:, 0], corners_3d[:, 1], corners_3d[:, 2], color=get_color_from_id(self.id), s=2)
        if name is None:
            name = r"$\{B_{" + str(self.id) + r"}\}$"
        plot_frame(self.pose.mat, name, axis_length=0.15, ax=ax)

    def get_3d_points_in_world(self) -> np.ndarray: 
        return self.checkerboard_geometry.get_3d_points_in_world(self.pose.mat)
    
    def get_internal_and_external_3d_points_in_world(self) -> np.ndarray: 
        return self.checkerboard_geometry.get_internal_and_external_3d_points_in_world(self.pose.mat)
