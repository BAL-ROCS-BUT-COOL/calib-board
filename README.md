# **calib-board**

A Python package for performing **external calibration** 📐 of multi-camera systems using synchronized images of a moving calibration pattern (e.g., chessboard or ChArUco) from **internally calibrated** 🔬 cameras.


## 📦 Contents

- 🏗️ [Installation](#%EF%B8%8F-installation)
- 📖 [How to Use](#-how-to-use)
    - 📚 [Preparation](#-preparation)
    - ⚡ [Running the Calibration](#-running-the-calibration)
    - 📊 [Output](#-output)
- 🎫 [License](#-license)
- 🤝 [Acknowledgments](#-acknowledgements)


## 🏗️ Installation
It is recommended to install this package through the external calibration meta-repository 👉`calibrate` to automatically get all the necessary dependencies! However, if this repository is installed standalone, the following steps are necessary:

1. Install dependecies:
    This package depends on the custom utilities Python package 👉`calib-commons`. Information on how to install it can be found in it's readme.

2. Clone the repo
    ```bash
    git clone https://github.com/BAL-ROCS-BUT-COOL/calib-board.git
    cd calib-board
    ```
3. Install the package and its requirements using `pip`:
    ```bash
    pip install -e .
    ```
    > Note: using the `-e` flag installs this package in editable mode (recommended). This means that changes made locally to the files are immediately reflected when importing the modules in other scripts.

4. Install additional dependencies (recommended):
    When using input videos for the board-calibration, `ffmpeg` is required. If not installed system wide anyways, the easiest way is to set up a conda environment for this package:
    ```bash
    conda create --name calibration python=3.11 ffmpeg
    conda activate calibration
    ```
    or alternatively, install `ffmpeg` in and existing conda environment
    ```bash
    conda install ffmpeg
    ```

## 📖 How to Use

This package contains all the functionality to perform external calibration with either a checkerboard or a charuco board. The workflow for both patterns is nearly identical - the only difference is that the parameters for running the main calibration script have to be set differently.

### 📚 Preparation

To run the calibration, two inputs are needed (details and recommendations regarding each input can be found in the subsequent sections):
- Parameters of the **internal calibration** for each individual camera (obtained using 👉`calib-commons`)
- **Synchronized videos or frame-series** from all cameras that include at least parts where the ckeckerboard is visible by multiple cameras at the same time (obtained using 👉`RocSync`).

> Note: The camera names have to match exactly between the intrinsics files and the videos / image-folders. Sometimes renaming is necessary, depending on how the input files are presented

**Camera Intrinsics (Internal Calibration Parameters)**

A folder containing all the internal calibration files can be specified via the `--intrinsics-path` parameter. The directory is expected to contain one json file for each camera-name (containing the intrinsics and distortion parameters) and has the following structure: 
```plaintext
intrinsics_directory/
├── <camera_name_1>_intrinsics.json
├── <camera_name_2>_intrinsics.json
└── ...
```

**Synchronized Videos / Frame-Series**

The synchronized footage serves as the main intput to the calibration. This can either be in the form of precisely synchronized videos (one video per camera) or synchronized frame-series (one folder of frames per camera). It is recommended to directly supply frame-series (the calibration will anyways need to extract frames from the videos, so this skips an unnecessary step). The following structure is expected for the input directory specified via the `--input-folder` parameter:

when using frame-series:
```plaintext
input_folder/
├── <camera_name_1>/
│   ├── 1.jpg
│   ├── 2.jpg
│   └── ...
├── <camera_name_2>/
│   ├── 1.jpg
│   ├── 2.jpg
│   └── ...
└── ...
```

when using videos:
```plaintext
input_folder/
├── <camera_name_1>.mp4
├── <camera_name_2>.mp4
└── ...
```
> Note: to avoid hard-to-detect errors later on, it is advised to quickly check the synchronized input for each camera. First of all, slight imprecision in the temporal synchronization can be detrimental and lead to a lot of outliers and weird results. Additionally, if any camera has moved even just slightly during the calibration sequence, it should be removed entirely, since its observations will be ambiguous and can "poison" the pose estimates of all other cameras. Finally it can be beneficial to remove cameras where visibility of the calibration board is very unstable/poor or all detections of the calibration board only cover a very small part of the field of view.


### ⚡ Running the Calibration

Once all the inputs have been prepared accordingly, the external calibration is performed by using the file `scripts\run.py`. A minimal example of the command used to start the calibration might be: 

```bash
python scripts/run.py --input-folder <path_to_images_or_videos> --intrinsics-folder <path_to_intrinsics_folder> --out-folder-calib <path_to_output_folder>
```
For more fine-grained control over the process, the follwing parameters are available, with a short explanation of what the purpose of each is:

**Help**
- `-h`: Show a help message listing all the parameters and a hint.

**Required**
- `--input-folder`: Path to the folder containing time-synchronized images or videos.
- `--intrinsics-path`: Path to the camera intrinsics parameter json files.

**Input/Output Params**
- `--out-folder-calib`: Name of the output folder for calibration results.
- `--sampling-step`: (only when using video as input) Stride for sampling frames from videos.
- `--start-time-window`: (only when using video as input) Start time for sampling (e.g., "00:00:00.000"). If None, starts from the beginning.
- `--end-time-window`: (only when using video as input) End time for sampling. If None, samples until the end.
- `--force-rerun / --no-force-rerun`: Delete all cached files/results and start fresh.

**Pre-Processing Params**
- `--board-type`: The type of calibration board (e.g., CHARUCO or CHESSBOARD).
- `--charuco-marker-size`: The size of the Aruco markers in a Charuco board [m].
- `--charuco-dictionary`: The dictionary used for the ChArUco board (e.g. 4X4_100).
- `--show-detection-images / --no-show-detection-images`: If True, display detection images during processing.
- `--save-detection-images / --no-save-detection-images`: If True, save detection images to the output folder.
- `--show-viz / --no-show-viz`: If True, display 3D visualizations of the results.
- `--save-viz / --no-save-viz`: If True, save 3D visualizations to the output folder.
- `--save-eval-metrics-to-json / --no-save-eval-metrics-to-json`: If True, save evaluation metrics to a JSON file.
- `--save-colmap-reconstruction / --no-save-colmap-reconstruction`: If True, save the reconstruction in a COLMAP-compatible format.

**Calibration Pattern Params**
- `--checkerboard-geometry.rows` : Number of internal corner rows (for Charuco and Chessboard!).
- `--checkerboard-geometry.columns` : Number of internal corner columns (for Charuco and Chessboard!).
- `--checkerboard-geometry.square-size`: The side length of a single square in meters (for Charuco and Chessboard!).

**Calibration Params**
- `--external-calibrator-config.checkerboard-motion`: The motion model for the checkerboard (PLANAR or FREE).
- `--external-calibrator-config.min-track-length`: The minimum number of camera views required for a 3D point to be considered valid.
- `--external-calibrator-config.reprojection-error-threshold`: The maximum reprojection error (in pixels) for an observation to be considered an inlier. This parameter is quite important to set right. If the threshold is to low, too many points are filtered and the calibration becomes invalid.
- `--external-calibrator-config.min-number-of-valid-observed-points-per-checkerboard-view`: The minimum number of detected corners required for a checkerboard view to be used. This can also be tuned; if set too high, a lot of otherwise usable detections are discarded. However an absolute minimum of 4 pionts is needed for PnP to succeed
- `--external-calibrator-config.camera-score-threshold`: A threshold for a camera's score to be considered well-calibrated (only "cosmetic").
- `--external-calibrator-config.ba-least-square-ftol`: The function tolerance for the bundle adjustment's least-squares optimizer.
- `--external-calibrator-config.least-squares-verbose`: Verbosity level for the least-squares solver.
- `--external-calibrator-config.free-first, --external-calibrator-config.no-free-first`: Whether to fix the first camera or not during the bundle adjustment. Not fixing it can improve the final result.
- `--external-calibrator-config.verbose`: General verbosity level for the calibrator.

> Note: when using `-h` there is one more class of parameters shown (`external-calibrator-config.checkerboard-geometry options`). However, these are automatically copied from the `calibration pattern params` anyways, so they do not have to be set specifically.



### 📊 Output
The main outputs generated by this package are the camera poses which are stored in the folder specified by `--out-folder-calib` in the form of a json file. Since the external calibration via calibration boards (of known size) is only defined up to a rigid body transformation, one camera is chosen as the origin and all the other poses are expressed relative to this origin.  

Additionally the following metrics are automatically generated and also stored in the output folder:

- `metrics.json`: In this file the most important metrics of the calibration for each individual camera can be found. `mean_eror` is the mean reprojection errors of all the observations of one camera. `mean_view_score` is a heuristic that quantifies how well distributed the observation points are in the camera's field of view (better distribution leads to more stable calibration, for more details see the [COLMAP Paper](https://demuc.de/papers/schoenberger2016sfm.pdf)). Lastly, `n_corres` is the number of correspondences (shared detections) each camera has. 
- `detections_2d.png`: This is a visualization of the reprojection of all the 3D calibration points (into the respectiv camera image plane). The colors encode whether a point was used for calibration (blue) or whether it was filtered due to being an outlier (red). These plots can be a good way for gauging how well distributed the detections are in the camera's field of view. 
- `reprojection_errors_2d.png`: These are the detailed reprojection errors (for each point and camera) in an x-y scatterplot.

> Note: A camera’s correspondences are the valid observations it makes of 3D points that are seen by at least one other camera. The collection of cameras that observe a given 3D point validly is called that point’s track. The number of such cameras is called "track length".


<table width="100%">
  <thead>
    <tr>
      <th width="33.33%"><code>metrics.json</code></th>
      <th width="33.33%"><code>detections_2d.png</code></th>
      <th width="33.33%"><code>reprojection_errors_2d.png</code></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td width="33.33%"><img src="./metrics.png"/></td>
      <td width="33.33%"><img src="./detections_2d.png"/></td>
      <td width="33.33%"><img src="./reprojection_errors_2d.png"/></td>
    </tr>
  </tbody>
</table>

> Note: the final achievable reprojection error depends a lot on factors like the image resolution, precision of the setup, image quality, small-scale movement of cameras, etc. As an example, for a setup with 4k GoPros that are calibrated via checkerboard and charuco board, the lowest reprojection error is around 0.3-0.7 pixels.  

## 🎫 License

This project is licensed under the **MIT License**. See the [LICENSE](./LICENSE) file for details.

## 🤝 Acknowledgements

TODO
