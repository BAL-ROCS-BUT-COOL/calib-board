# **calib-board**

A Python package for performing **external calibration** 📐📷 of multi-camera systems using synchronized images⏱️ of a moving calibration pattern 🏁 (e.g., chessboard or ChArUco) from internally calibrated cameras 📷.


---


## Table of Contents 📋
- [Installation](#installation) 🔧
- [Documentation](#-documentation) 🗎
- [How To Use](#how-to-use) ❔
- [License](#-license) 📃
- [Acknowledgments](#)


## **Installation** 🔧



### Dependencies (calib-commons) ⛓️
The package depends on the custom utilities Python package 🧰 [`calib-commons`](https://github.com/tflueckiger/calib-commons). To install it:

   ```bash
   # clone the repos
   git clone https://github.com/tflueckiger/calib-commons.git
   cd calib-commons
   # install the package with pip
   pip install .
   # optional: install in editable mode:
   pip install -e . --config-settings editable_mode=strict
   ```

> ⛓️ Dependencies : if the additional dependencies listed in requirements.txt are not satisfied, they will be automatically installed from PyPi. 

### Installation of calib-board

 ```bash
   # clone the repos
   git clone https://github.com/tflueckiger/calib-board.git
   cd calib-board
   # install the package with pip
   pip install .
   # optional: install in editable mode:
   pip install -e . --config-settings editable_mode=strict
   ```
---

### Additional dependencies

For video functionality, ``ffmpeg` is required. If not installed system-wide already, the easiest way by setting up everything in a `conda` environment and install using

```bash
conda install ffmpeg
```

## **Documentation** 🗎

### 📝 **Prequisites**
Ensure you have the following before running the package:

1. **Synchronized Images/Videos from All Cameras** ⏱️📷
2. **Internal Camera Parameters (Intrinsics)** ⚙️📷

###  **1a. Synchronized Images** ⏱️📷

Organize synchronized images in a directory with the following structure:

```plaintext
input_folder/
├── camera1/
│   ├── 1.jpg
│   ├── 2.jpg
│   └── ...
├── camera2/
│   ├── 1.jpg
│   ├── 2.jpg
│   └── ...
└── cameraN/
    ├── 1.jpg
    ├── 2.jpg
    └── ...
```

- Each subdirectory (`camera1`, `camera2`, ..., `cameraN`) contains images for a single camera.
- Filenames **must match across cameras** to ensure synchronization (e.g., `1.jpg` in `camera1` corresponds to `1.jpg` in `camera2`).

###  **1b. Synchronized Videos** ⏱️📷

Alternatively, you can organize the folder using synchronized videos of all cameras:

```plaintext
input_folder/
├── camera1.mp4
├── camera2.mp4
├── ...
└── cameraN
```
¨
This will accordingly extract frames using `ffmpeg` at the start of the main script and create the folder structure from above. Please make sure that `ffmpeg` is installed.

### **2. Camera Intrinsics** ⚙️📷

Place intrinsics for each camera in a directory like this, making sure that the filenames match with the camera names in the input folder:

```plaintext
intrinsics_directory/
├── camera1_intrinsics.json
├── camera2_intrinsics.json
└── ...
```

>💡 This can be done using using the [`calib-commons`](https://github.com/tflueckiger/calib-commons) toolbox, which includes a ['calibrate-intrinsics'](https://github.com/tflueckiger/calib-commons?tab=readme-ov-file#calibrate-intrinsics) command-line tool for automatic internal calibration of multiple cameras using either videos recordings or images. The tool creates a folder containing camera intrinsics in JSON files matching the required format. See here for documentation on how to generate intrinsics with ['calibrate-intrinsics'](https://github.com/tflueckiger/calib-commons?tab=readme-ov-file#calibrate-intrinsics).

---

## ❔ **How To Use**

Once [prerequisites](#-prequisites) are met, perform external calibration:

```bash
python scripts/run.py --input-folder <path_to_images_or_videos> --intrinsics-folder <path_to_intrinsics_folder>
```

For processing time-synchronized videos, use the `--start_time_window` and ``--end_time_window` flags as well as `--sampling_step` to process only a subset of frames (i.e., where the calibration board is visible) and speed up the procedure.


```bash
python scripts/run.py --input-folder <path_to_images_or_videos> --intrinsics-folder <path_to_intrinsics_folder> --start_time_window 00:32:00.0 --end_time_window 00:35:15.0 --sampling_step 10
```

For information on all configuration parameters, run `python scripts/run.py -h`.


### Output 📤
The camera poses 📐are saved in ***results/camera_poses.json***.

In addition, the following **metrics** 🎯, per camera and overall, are saved in ***results/metrics.json***: 
- mean **reprojection error**
- standard deviation of the reprojection error
- view score of the cameras for the calibration dataset (score introduced and used by [COLMAP](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf))
- number of correspondences for each cameras 

> The *number of correspondences* of a camera corresponds to the number of conform observations a camera has of object (=3D) points with a track length higher or equal to 2. 

> The *track* of an object (=3D) point is the set of cameras in which the point is observed, and for which the observation is conform.

 

<figure style="text-align: center;">
    <img src="https://github.com/user-attachments/assets/238b9ebf-ac48-402e-b448-75916ae1068c" alt="Texte alternatif" style="width: 50%">
</figure>
<!-- 
<figure style="text-align: center;">
    <img src="https://github.com/user-attachments/assets/db46b25b-18d9-4d9a-842d-007b8ee50a25" alt="Texte alternatif" style="width: 50%">
</figure>
<figure style="text-align: center;">
    <img src="https://github.com/user-attachments/assets/6a8b1495-f0c8-44b2-9c39-63f5c33c41a5" alt="Texte alternatif" style="width: 50%">
</figure>
-->

| Reprojections and observations                           | Reprojection errors                            |
| ----------------------------------- | ----------------------------------- |
| ![Description de l'image 1](https://github.com/user-attachments/assets/db46b25b-18d9-4d9a-842d-007b8ee50a25)  | ![Description de l'image 2](https://github.com/user-attachments/assets/6a8b1495-f0c8-44b2-9c39-63f5c33c41a5)  |
---

---

## 📃 **License**

This project is licensed under the **MIT License**. See the [LICENSE](https://github.com/tflueckiger/calib-board/blob/main/LICENSE) file for details.

---

## **Acknowledgments**

TODO
