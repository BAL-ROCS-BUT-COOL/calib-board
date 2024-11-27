
# calib-board

**calib-board** .

## Features

- **ddd**: ddd.

## Installation

### 1. Clone the Repository

Clone the `calib-board` repository from GitHub:

```bash
git clone https://github.com/tflueckiger/calib-board.git
cd calib-board
pip install .
```

Note: this command will automatically trigger the install of the dependencies from PyPi if not satisfied (see requirements.txt)
In addition, this package uses utilities from the custom package calib-commons (https://github.com/tflueckiger/calib-commons), which is also automatically installed.

If you want to install the package in the editable mode, add the option:
```bash
pip install -e . --config-settings editable_mode=strict
```

## Usage

The toolbox requires:

1. **Synchronized images from all cameras**
2. **Internal parameters (=intrinsics) for each camera**

**1. Synchronized Images from All Cameras**

Ensure that your images are temporally synchronized and organized within a directory as follows:

```
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

Each subdirectory (`camera1`, `camera2`, ..., `cameraN`) corresponds to a different camera and contains its respective images. The filenames should be consistent across all camera folders to reflect the synchronization. For example, `1.jpg` in `camera1` should correspond to `1.jpg` in `camera2`, and so on.

**2. Intrinsic Parameters for Each Camera**

Provide the intrinsic parameters for each camera in a structured format, such as JSON or YAML. These files should be named to correspond with their respective camera directories and placed in a directory :

```
intrinsics_directory/
│   ├── camera1_intrinsics.json
│   ├── camera2_intrinsics.json
│   └── ...
│   └── cameraN_intrinsics.json
```


The main script to run the calibration is scripts/run.py


First, modify the pre-processing parameters (board detection) and calibration parameters in the section USER INTERFACE. 



## License

This project is licensed under the MIT License. See the [LICENSE](https://github.com/your-username/boardCal/blob/main/LICENSE) file for details.

## Acknowledgments


