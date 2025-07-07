"""
This module provides functions for visualizing 3D calibration scenes and 2D
observations using matplotlib.
"""

import math
import os
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams
from matplotlib.axes import Axes

# Local application/library specific imports
from calib_board.core.correspondences import Correspondences
from calib_board.core.scene import SceneCheckerboard, SceneType
from calib_commons.camera import Camera
from calib_commons.types import idtype

# --- Module Constants ---

MODIFIERS: dict[SceneType, str] = {
    SceneType.SYNTHETIC: r"\overline",
    SceneType.ESTIMATE: r"\hat",
}

MARKERS_OBSERVATIONS: dict[SceneType, str] = {
    SceneType.SYNTHETIC: "+",
    SceneType.ESTIMATE: "x",
}


def _get_scene_coords(
    scene: SceneCheckerboard,
) -> tuple[list[float], list[float], list[float]]:
    """
    Extracts all x, y, z coordinates from cameras and checkerboard corners in a scene.

    Args:
        scene: The scene to process.

    Returns:
        A tuple containing three lists: x_coords, y_coords, and z_coords.
    """
    x, y, z = [], [], []

    for camera in scene.cameras.values():
        x.append(camera.pose.get_x())
        y.append(camera.pose.get_y())
        z.append(camera.pose.get_z())

    for checker in scene.checkers.values():
        corners = checker.get_3d_points_in_world()
        x.extend(corners[:, 0])
        y.extend(corners[:, 1])
        z.extend(corners[:, 2])

    return x, y, z


def init_ax(ax: Axes) -> None:
    """
    Initializes a 3D axes object with default labels.

    Note: This helper function is currently not used by other functions in this module.

    Args:
        ax: The matplotlib 3D axes object to initialize.
    """
    ax.set_aspect("auto")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")


def plot_scene(scene: SceneCheckerboard, ax: Axes, show_ids: bool = False) -> None:
    """
    Plots the cameras and checkerboards of a single scene onto a 3D axis.

    Args:
        scene: The scene to plot.
        ax: The matplotlib 3D axes object to plot on.
        show_ids: If True, display the ID labels for cameras and checkerboards.
    """
    mod = MODIFIERS[scene.type]
    
    for camera in scene.cameras.values(): 
        name = r"$\{" + mod + r"{C}_{" + str(camera.id) + r"}\}$"
        camera.plot(name)

    for checker in scene.checkers.values(): 
        if show_ids:
            name = r"$\{" + mod + r"{B}_{" + str(checker.id) + r"}\}$"
        else: 
            name = ""
        checker.plot(name)


def visualize_scenes(
    scenes: list[SceneCheckerboard],
    show_ids: bool = False,
    show_fig: bool = True,
    save_fig: bool = False,
    save_path: Optional[str] = None,
    elev: Optional[float] = None,
    azim: Optional[float] = None,
) -> None:
    """
    Visualizes one or more 3D scenes on a single plot.

    Args:
        scenes: A list of `SceneCheckerboard` objects to visualize.
        show_ids: If True, display ID labels for scene elements.
        show_fig: If True, display the plot window.
        save_fig: If True, save the plot to a file.
        save_path: The file path for saving the figure. Required if save_fig is True.
        elev: The elevation angle for the 3D plot view.
        azim: The azimuth angle for the 3D plot view.
    """
    if not scenes:
        print("Warning: No scenes provided to visualize.")
        return

    fig = plt.figure(figsize=(8, 8))
    rcParams["text.usetex"] = True
    ax = fig.add_subplot(projection="3d")

    if elev is not None and azim is not None:
        ax.view_init(elev=elev, azim=azim)

    # Determine bounding box to fit all scenes
    all_x, all_y, all_z = [], [], []
    for scene in scenes:
        x, y, z = _get_scene_coords(scene)
        all_x.extend(x)
        all_y.extend(y)
        all_z.extend(z)

    if not all_x:
        print("Warning: Scenes contain no elements to visualize.")
        return

    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    z_min, z_max = min(all_z), max(all_z)

    # Create a cubic bounding box for consistent aspect ratio
    x_mid, y_mid, z_mid = (x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    half_range = max_range / 2
    ax.set_xlim(x_mid - half_range, x_mid + half_range)
    ax.set_ylim(y_mid - half_range, y_mid + half_range)
    ax.set_zlim(z_mid - half_range, z_mid + half_range)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("3D Scenes", fontsize=14, pad=20)

    for scene in scenes:
        plot_scene(scene, ax, show_ids)

    if save_fig:
        if not save_path:
            raise ValueError("A 'save_path' must be provided when 'save_fig' is True.")
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        fig.savefig(save_path, dpi=300, bbox_inches="tight")

    if show_fig:
        plt.show(block=False)


def _plot_point_group(ax: Axes, points_dict: dict, marker: str, color: str) -> None:
    """Helper to plot a dictionary of point groups."""
    for _2d in points_dict.values():
        if _2d.size > 0:
            ax.plot(
                _2d[:, 0], _2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color
            )


def visualize_2d(
    scene: SceneCheckerboard,
    observations: Correspondences,
    which: str = "both",
    show_fig: bool = True,
    save_fig: bool = False,
    save_path: Optional[str] = None,
    show_ids: bool = False,
) -> None:
    """
    Visualizes the 2D observations and reprojections for each camera.

    Colors are used to distinguish point types:
    - Blue: Conform points (good reprojection, part of a good board observation).
    - Red: Non-conform points (bad reprojection).
    - Orange: Points with good reprojection but part of a discarded board observation.

    Args:
        scene: The scene containing cameras and reprojection data.
        observations: The observed 2D points.
        which: Which points to show ("conform", "non-conform", or "both").
        show_fig: If True, display the plot window.
        save_fig: If True, save the plot to a file.
        save_path: The file path for saving the figure.
        show_ids: If True, display checkerboard ID labels on the plot.
    """
    camera_ids = scene.cameras.keys()
    if not camera_ids:
        print("Warning: No cameras in the scene to visualize.")
        return

    n = len(camera_ids)
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(cols * 4, rows * 3), squeeze=False)
    axs = axs.flatten()

    for i, cam_id in enumerate(camera_ids):
        ax = axs[i]
        cam = scene.cameras[cam_id]
        res_x, res_y = cam.intrinsics.resolution
        ax.set_xlim(0, res_x)
        ax.set_ylim(res_y, 0)
        ax.set_aspect("equal", "box")
        ax.set_title(f"Observations in Camera {cam_id}")

        # --- Data Preparation ---
        obs_conform, obs_nonconform, obs_discarded = {}, {}, {}
        rep_conform, rep_nonconform, rep_discarded = {}, {}, {}

        for chk_id, obs in observations[cam_id].items():
            if obs._2d is None:
                continue

            mask_not_nan = ~np.isnan(obs._2d).any(axis=1)
            conform_mask = obs._conformity_mask

            if obs._is_conform:
                obs_conform[chk_id] = obs._2d[conform_mask & mask_not_nan]
                obs_nonconform[chk_id] = obs._2d[~conform_mask & mask_not_nan]
                if scene.reprojections and scene.reprojections[cam_id].get(chk_id):
                    rep = scene.reprojections[cam_id][chk_id]
                    if rep._2d is not None:
                        rep_conform[chk_id] = rep._2d[conform_mask & mask_not_nan]
                        rep_nonconform[chk_id] = rep._2d[~conform_mask & mask_not_nan]
            else:
                obs_nonconform[chk_id] = obs._2d[~conform_mask & mask_not_nan]
                obs_discarded[chk_id] = obs._2d[conform_mask & mask_not_nan]
                if scene.reprojections and scene.reprojections[cam_id].get(chk_id):
                    rep = scene.reprojections[cam_id][chk_id]
                    if rep._2d is not None:
                        rep_nonconform[chk_id] = rep._2d[~conform_mask & mask_not_nan]
                        rep_discarded[chk_id] = rep._2d[conform_mask & mask_not_nan]

        # --- Plotting ---
        if which in ("both", "conform"):
            _plot_point_group(
                ax, obs_conform, MARKERS_OBSERVATIONS[SceneType.SYNTHETIC], "blue"
            )
            _plot_point_group(
                ax, rep_conform, MARKERS_OBSERVATIONS[SceneType.ESTIMATE], "blue"
            )

        if which in ("both", "non-conform"):
            _plot_point_group(
                ax, obs_nonconform, MARKERS_OBSERVATIONS[SceneType.SYNTHETIC], "red"
            )
            _plot_point_group(
                ax, rep_nonconform, MARKERS_OBSERVATIONS[SceneType.ESTIMATE], "red"
            )
            _plot_point_group(
                ax, obs_discarded, MARKERS_OBSERVATIONS[SceneType.SYNTHETIC], "orange"
            )
            _plot_point_group(
                ax, rep_discarded, MARKERS_OBSERVATIONS[SceneType.ESTIMATE], "orange"
            )

        if show_ids:
            for chk_id, obs in observations[cam_id].items():
                if obs._2d is not None and obs._2d.size > 0:
                    first_valid_point = next(
                        (p for p in obs._2d if not np.isnan(p).any()), None
                    )
                    if first_valid_point is not None:
                        ax.text(first_valid_point[0], first_valid_point[1], str(chk_id))

    # Turn off unused subplots
    for j in range(n, len(axs)):
        axs[j].axis("off")

    fig.tight_layout()

    if save_fig:
        if not save_path:
            raise ValueError("A 'save_path' must be provided when 'save_fig' is True.")
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        fig.savefig(save_path, dpi=300, bbox_inches="tight")

    if show_fig:
        plt.show(block=False)


def plot_reprojections_in_camera(
    correspondences: Correspondences, camera_id: idtype, ax: Axes
) -> None:
    """
    Plots reprojected points for a specific camera.

    Note: The functionality of this helper is now integrated into `visualize_2d`.
    It is kept for potential legacy use.

    Args:
        correspondences: The dictionary of all correspondences.
        camera_id: The ID of the camera whose view is to be plotted.
        ax: The matplotlib axes object to plot on.
    """
    for checker_id, observation in correspondences[camera_id].items():
        if observation and observation._is_conform and observation._2d is not None:
            marker = MARKERS_OBSERVATIONS[SceneType.ESTIMATE]
            ax.plot(
                observation._2d[:, 0],
                observation._2d[:, 1],
                marker,
                markersize=4,
                markeredgewidth=1,
                color="blue",
                alpha=1.0,
            )
            ax.text(
                observation._2d[0, 0],
                observation._2d[0, 1],
                str(checker_id),
                color="black",
                fontsize=10,
                alpha=1,
            )


def plot_observations_in_camera(
    correspondences: Correspondences, camera_id: idtype, ax: Axes
) -> None:
    """
    Plots observed points for a specific camera.

    Note: The functionality of this helper is now integrated into `visualize_2d`.
    It is kept for potential legacy use.

    Args:
        correspondences: The dictionary of all correspondences.
        camera_id: The ID of the camera whose view is to be plotted.
        ax: The matplotlib axes object to plot on.
    """
    for checker_id, observation in correspondences[camera_id].items():
        if observation and observation._is_conform and observation._2d is not None:
            marker = MARKERS_OBSERVATIONS[SceneType.SYNTHETIC]
            ax.plot(
                observation._2d[:, 0],
                observation._2d[:, 1],
                marker,
                markersize=4,
                markeredgewidth=1,
                color="black",
                alpha=0.5,
            )
            ax.text(
                observation._2d[0, 0],
                observation._2d[0, 1],
                str(checker_id),
                color="black",
                fontsize=10,
                alpha=1,
            )


# The following block of code was commented out in the original file and is
# preserved here in its original state for reference. It appears to be an
# incomplete or legacy implementation for plotting reprojection error vectors.
#
# def plot_reprojection_errors(scene_estimate: SceneCheckerboard,
#                              observations: Correspondences,
#                              show_fig = True,
#                             save_fig = False,
#                             save_path = None) -> None:
#
#     n = len(scene_estimate.cameras)
#     cols = math.ceil(math.sqrt(n))
#     rows = math.ceil(n/ cols)
#
#     fig_width = cols * 4  # Each subplot has a width of 4 inches (adjust as needed)
#     fig_height = rows * 4  # Each subplot has a height of 3 inches (adjust as needed)
#
#     # Create a figure with dynamic size
#     fig, axs = plt.subplots(rows, cols, figsize=(fig_width, fig_height))
#
#     # fig, axs = plt.subplots(rows, cols)
#     axs = np.array(axs).reshape(-1) if n > 1 else [axs]
#     axsUsed = [False for ax in axs]
#
#     axIndex = 0
#
#     all_errors = []
#     all_errors_conform = []
#
#     for camera in scene_estimate.cameras.values():
#         ax = axs[axIndex]
#         axsUsed[axIndex] = True
#         axIndex += 1
#
#         # errors_conform = get_errors_x_y_in_cam(camera, scene_estimate.object_points, observations, which="conform")
#         # errors_nonconform = get_errors_x_y_in_cam(camera, scene_estimate.object_points, observations, which="non-conform")
#         errors_conform = get_errors_x_y_in_cam(camera, scene_estimate.checkers, observations, which="conform")
#         errors_nonconform = get_errors_x_y_in_cam(camera, scene_estimate.checkers, observations, which="non-conform")
#         # print(errors_conform)
#         # print(errors_nonconform)
#         # errors_conform = list(errors_conform.values())
#         # errors_nonconform = list(errors_nonconform.values())
#         errors_tot = np.concatenate(errors_conform+errors_nonconform, axis=0)        # plt.xlim(-plot_lim, plot_lim)
#
#
#         all_errors.extend(errors_tot)
#         all_errors_conform.extend(errors_conform)
#
#         # error_min = np.nanmin(errors_tot)
#         # error_max = np.nanmax(errors_tot)
#
#         # plot_lim = max(abs(error_min), abs(error_max))
#
#         # # fig = plt.figure()
#         # # plt.xlim(-plot_lim, plot_lim)
#         # # plt.ylim(-plot_lim, plot_lim)
#         # ax.set_xlim([-plot_lim, plot_lim])
#         # ax.set_ylim([-plot_lim, plot_lim])
#         # ax = plt.gca()
#         aspect_ratio = 1
#         ax.set_aspect(aspect_ratio / ax.get_data_ratio())
#
#
#         for errors_ in errors_conform:
#             ax.scatter(errors_[:,0], errors_[:,1], s=0.5, c='blue', marker = 'o', alpha=1)
#
#         for errors_ in errors_nonconform:
#             ax.scatter(errors_[:,0], errors_[:,1], s=0.5, c='red', marker = 'o', alpha=1)
#
#         circle = plt.Circle((0,0), 1, edgecolor='red', facecolor='none', linewidth=1.5, alpha = 1)
#         ax.add_patch(circle)
#
#         circle = plt.Circle((0,0), 0.7, edgecolor='red', facecolor='none', linewidth=1.5, alpha = 1)
#         ax.add_patch(circle)
#
#         title = f"Repr. errors in {camera.id}"
#         ax.set_title(title)
#         # plt.title(title)
#
#     # compute axis limits (same for all subplots)
#     # error_min = np.nanmin(all_errors)
#     # error_max = np.nanmax(all_errors)
#     # plot_lim = max(abs(error_min), abs(error_max))
#     error_min = np.nanmin(all_errors_conform)
#     error_max = np.nanmax(all_errors_conform)
#     # plot_lim = max(abs(error_min), abs(error_max))
#     plot_lim = 2
#     axIndex = 0
#     for camera in scene_estimate.cameras.values():
#         ax = axs[axIndex]
#         axIndex += 1
#         ax.set_xlim([-plot_lim, plot_lim])
#         ax.set_ylim([-plot_lim, plot_lim])
#
#         if save_fig:
#             # Create the directory if it does not exist
#             os.makedirs(os.path.dirname(save_path), exist_ok=True)
#             fig.savefig(save_path, dpi=300, bbox_inches='tight')
#         if show_fig:
#             plt.show(block=False)