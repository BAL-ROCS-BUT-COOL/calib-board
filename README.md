# **Calib-Board**

A Python package for performing **external calibration** of multi-camera systems using synchronized images of a moving calibration pattern (e.g., chessboard or ChArUco) from internally calibrated cameras.

---


## **Installation**

1. Clone the repository:

   ```bash
   git clone https://github.com/tflueckiger/calib-board.git
   cd calib-board
   pip install .
   ```

2. Optional: Install in editable mode:

   ```bash
   pip install -e . --config-settings editable_mode=strict
   ```

> Dependencies, including utilities from [`calib-commons`](https://github.com/tflueckiger/calib-commons), are automatically installed.

---

## **Prerequisites**

Ensure you have the following before running the package:

1. **Synchronized Images from All Cameras**  
2. **Internal Camera Parameters (Intrinsics)**  

### **1. Synchronized Images**

Organize synchronized images in a directory with the following structure:

```plaintext
images_directory/
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

### **2. Camera Intrinsics**

Place intrinsics for each camera in a directory like this:

```plaintext
intrinsics_directory/
├── camera1_intrinsics.json
├── camera2_intrinsics.json
└── ...
```

Generate intrinsics using the `calib-commons` toolbox, which includes a `calibrate-intrinsics` command-line tool for automatic internal calibration.

---

## **Generating Intrinsics with Calib-Commons**

### **Option 1: Using Video Recordings**

1. Place video recordings of a moving chessboard in a common directory:

   ```plaintext
   data_directory/
   ├── camera1.mp4
   ├── camera2.mp4
   └── ...
   ```

2. Navigate to the directory:

   ```bash
   cd path_to_data_directory
   ```

3. Run the calibration command:

   ```bash
   calibrate-intrinsics --use_videos --square_size 0.03 --chessboard_width 11 --chessboard_height 8 --sampling_step 45
   ```

---

### **Option 2: Using Images**

1. Organize images into subfolders, one per camera:

   ```plaintext
   data_directory/
   ├── camera1/
   │   ├── frame1.jpg
   │   ├── frame2.jpg
   │   └── ...
   ├── camera2/
   │   ├── frame1.jpg
   │   ├── frame2.jpg
   │   └── ...
   └── cameraN/
       ├── frame1.jpg
       ├── frame2.jpg
       └── ...
   ```

2. Navigate to the directory:

   ```bash
   cd path_to_data_directory
   ```

3. Execute the calibration command:

   ```bash
   calibrate-intrinsics --square_size 0.03 --chessboard_width 11 --chessboard_height 8
   ```

> For detailed usage, run `calibrate-intrinsics --help`.

---

## **Usage**

Once prerequisites are met, perform external calibration:

1. Edit the user interface parameters in `scripts/run.py`:
   - **Board Detection Parameters**: Configure settings for detecting the calibration pattern.
   - **Calibration Algorithm Parameters**: Adjust algorithm settings.

2. Run the script:

   ```bash
   python scripts/run.py
   ```

---

## **License**

This project is licensed under the **MIT License**. See the [LICENSE](https://github.com/tflueckiger/calib-board/blob/main/LICENSE) file for details.

---

## **Acknowledgments**

TODO