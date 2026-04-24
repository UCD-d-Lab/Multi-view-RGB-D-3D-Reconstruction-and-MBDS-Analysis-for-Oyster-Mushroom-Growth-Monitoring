
---

# Multi-view RGB-D 3D Reconstruction and MBDS Analysis for Oyster Mushroom Growth Monitoring

This repository contains Python scripts designed for automated RGB-D acquisition, multi-view point cloud reconstruction, mushroom baseline differential segmentation (MBDS), and the estimation of size and volume for monitoring the growth of oyster mushroom clusters.

The code accompanies the manuscript:

**Multi-view 3D reconstruction with mushroom baseline differential segmentation for oyster mushroom growth phenotyping and harvest-window prediction**

Yuqiao Ren¹, Dimitrios Argyropoulos¹\**

¹ Digital Agriculture & Bioresource (d-Lab), School of Biosystems and Food Engineering, University College Dublin, Dublin, Ireland  

\* Corresponding author: dimitrios.argyropoulos@ucd.ie
---

## Overview

Accurate and continuous monitoring of oyster mushroom growth is important for phenotyping, growth-rate analysis, and harvest-window prediction. Conventional manual measurements are time-consuming and may disturb the cultivation process, while 2D image-based methods are limited by self-occlusion and incomplete spatial information in complex mushroom clusters. This repository provides the Python scripts used to implement a **multi-view RGB-D point-cloud reconstruction and analysis pipeline** for oyster mushroom growth monitoring. The pipeline integrates automated RealSense RGB-D acquisition, camera ROI setting, multi-view point-cloud reconstruction, pass-through filtering (PF), mushroom baseline differential segmentation (MBDS), AABB-based dimension estimation, and voxel-based volume estimation. The workflow is designed to support continuous, non-destructive monitoring of oyster mushroom clusters using fixed Intel RealSense RGB-D cameras. Reconstruction configurations with various camera model and number can be evaluated by changing the selected calibration bags and camera subset.

<p align="center">
  <img src="imgs/pipeline_overview.png" width="700">
</p>

<p align="center">Overview of the multi-view RGB-D acquisition, PF reconstruction, MBDS segmentation, dimension & volume estimation pipeline.</p>

Experiments reported in the manuscript show that the proposed pipeline can estimate oyster mushroom cluster dimensions and volume with satisfactory agreement against reference measurements, and can provide time-series growth curves for high accurate logistic growth modelling and harvest-time prediction. The same processing framework can be adapted to different camera subsets, including the three-camera and five-camera configurations evaluated in the manuscript.

---

## Script description

`roi_setting.py`: Interactive tool for manually defining the auto-exposure region of interest (ROI) for each RealSense camera. The ROI is saved to `roi_config.json`. 
`automatic_rgbd_capture.py`: Automated RGB-D time-lapse acquisition script. It detects connected RealSense cameras, loads camera-specific JSON settings, applies the ROI configuration, performs warm-up, and records one `.bag` file per camera per acquisition round.
`watchdog_runner.py`: Optional watchdog script for unattended acquisition. It restarts the acquisition script if the process exits unexpectedly and can support heartbeat-based monitoring. 
`pcd_pf_reconstruction.py`: Performs chessboard-based multi-camera calibration, pass-through filtering, RGB-D point cloud generation, multi-view transformation, merging, cropping, and PCD export. 
`dimension_volume_calculation.py`: Performs MBDS segmentation, baseline-guided ICP alignment, refined differencing, footprint cap construction, voxel-based volume estimation, and AABB-based dimension estimation. 
`bag_manger.py`: Helper module for reading RealSense `.bag` files and extracting depth, infrared, and aligned colour frames. 
`calculate_rmsd_kabsch.py`: Kabsch algorithm and RMSD calculation module used for chessboard-based calibration. 
`realsense_device_manager.py`: RealSense helper module for device management and frame polling. It is retained as a helper/reference utility. 

## Environment

The scripts were developed for Intel RealSense RGB-D cameras and Python-based point-cloud processing. The code has been used in the manuscript with the following general environment:

- Python 3.12.9
- Intel RealSense SDK 2.0
- `pyrealsense2`
- Open3D
- OpenCV
- NumPy
- SciPy

Note:** `pyrealsense2` requires the Intel RealSense SDK and compatible camera drivers. Installation and camera access may depend on the operating system, USB bandwidth, camera model, and RealSense firmware version.

## Workflow

### 1. ROI setting for auto-exposure

Run the ROI setting script before long-term acquisition:

```bash
python roi_setting.py --target depth --outfile roi_config.json
```

For each connected camera, drag a rectangle over the expected mushroom growth region. The selected ROI is saved in JSON format. This ROI is later used by `automatic_rgbd_capture.py` to set the auto-exposure region for each camera.

---

### 2. Automated RGB-D acquisition

Update the following paths in `automatic_rgbd_capture.py` before running:

```python
OUTPUT_DIR = "Path to Output Directory"
SETTINGS_JSON_BY_MODEL = {
    "D405":  "Path to Camera Setting JSON for D405",
    "D435i": "Path to Camera Setting JSON for D435i",
}
ROI_JSON = "Path to ROI Configuration JSON"
```

Then start time-lapse acquisition:

```bash
python automatic_rgbd_capture.py
```

Default acquisition settings:

- Acquisition interval: 15 min 
- Recording duration per camera: 1.0 s
- Warm-up frames: 30 
- Resolution: 1280 × 720 
- Capture mode: Sequential
- Streams: Colour, depth, infrared-1

The script saves one `.bag` file per camera per acquisition round using the following folder structure:

```text
Data_Output/
├── <camera_serial_1>/
│   └── YYYY-MM-DD/
│       ├── YYYY-MM-DD_HH-MM-SS.bag
│       └── ...
├── <camera_serial_2>/
│   └── YYYY-MM-DD/
│       ├── YYYY-MM-DD_HH-MM-SS.bag
│       └── ...
└── ...
```
---

### 3. Optional watchdog runner

For unattended long-term acquisition, the acquisition script can be started through:

```bash
python watchdog_runner.py
```

Before use, update the target script and working directory in `watchdog_runner.py`:

```python
TARGET_SCRIPT = "automatic_rgbd_capture.py"
WORK_DIR = pathlib.Path("path/to/script/directory")
```

The watchdog script records terminal output to a log folder and restarts the acquisition script if it exits unexpectedly. Heartbeat-based detection of unresponsive processes is optional and requires the acquisition script to update `pcd_heartbeat.txt` periodically.

---

### 4. Multi-view point-cloud reconstruction

Before running `pcd_pf_reconstruction.py`, update:

```python
CAPTURE_ROOT = "path/to/capture/root"

bag_files = [
    "path/to/chessboard1.bag",
    "path/to/chessboard2.bag",
    "path/to/chessboard3.bag",
    "path/to/chessboard4.bag",
    "path/to/chessboard5.bag",
]

save_path = "path/to/output/directory/Generated_PCD"
ref_serial = "serial_number_for_ref_camera"
```

Then run:

```bash
python pcd_pf_reconstruction.py
```
The script can be adapted for different camera layouts, update the `bag_files` list and ensure that `ref_serial` matches the selected reference camera used for the manuscript coordinate system.

The script performs:

1. infrared chessboard corner detection;
2. depth-based 3D corner back-projection;
3. Kabsch-based rigid transformation estimation;
4. RealSense RGB-D frame reading and colour-to-depth alignment;
5. depth filtering using a working range of 0.3–0.6 m;
6. colour filtering using HSV, Lab, and RGB threshold masks;
7. morphological mask refinement;
8. point-cloud generation for each camera frame;
9. transformation into the selected reference-camera coordinate frame;
10. multi-view merging, spatial cropping, and voxel down-sampling.

The output files are saved as:

```text
Generated_PCD_YYYY-MM-DD_HH-MM-SS.pcd
```

---

### 5. MBDS segmentation and dimension/volume estimation

Before running `dimension_volume_calculation.py`, update:

```python
BASELINE_PATH = "Path to Baseline PCD"
CURRENT_DIR = "Path to Batch Directory"
OUT_CSV_PATH = "Path to Output CSV File"
OUT_DEBUG_DIR = "Path to Debug Directory"
OUT_PCD_DIR = "Path to Output PCD Directory"
```

Then run:

```bash
python dimension_volume_calculation.py
```

The script processes each reconstructed PCD against a baseline PCD acquired before visible mushroom emergence. The main steps are:

1. initial temporal Euclidean differencing;
2. baseline-guided ICP registration;
3. refined differencing after alignment;
4. DBSCAN-based dominant-cluster extraction;
5. baseline footprint-cap reconstruction;
6. voxelization, morphological refinement, and column filling;
7. AABB-based dimension estimation from the refined voxel mask;
8. CSV export of size and volume measurements.

The output CSV contains:

```text
file
timestamp
fitness
rmse
bbox_dx_mm
bbox_dy_mm
bbox_dz_mm
solid_cm3
floor_cm3
columns_only_cm3
total_cm3
volume_mL
```

In the manuscript, the column-filled volume is used as the final mushroom volume estimate. 

---

## Results

The pipeline was evaluated using oyster mushroom cultivation sequences acquired with fixed multi-view Intel RealSense RGB-D cameras. The reconstructed point clouds were processed using PF reconstruction and MBDS segmentation before dimension and volume estimation.

### Validation against reference measurements

The proposed RGB-D point-cloud analysis pipeline was compared against manually processed LiDAR reference point clouds. Both three-camera and five-camera reconstruction configurations achieved satisfactory agreement for mushroom cluster width, depth, height, and volume estimation.

<p align="center">
  <img src="imgs/validation_results.png" width="700">
</p>

<p align="center">Validation plots comparing RGB-D-based estimates with portable LiDAR-based reference measurements.</p>

---

### Time-series mushroom growth monitoring

The estimated height, width, depth, and volume trajectories were used to monitor oyster mushroom cluster development across cultivation cycles. The original 15-minute measurements were downsampled to 1-hour intervals for time-series visualisation.

<p align="center">
  <img src="imgs/growth_timeseries.png" width="700">
</p>

<p align="center">Example time-series estimates of oyster mushroom cluster dimensions and volume.</p>

---

### Growth modelling and harvest-window prediction

The processed time-series volume measurements were further used for logistic growth modelling based on exported excel file. The first and second derivatives of the fitted logistic curves were used to evaluate growth rate, growth acceleration, maturity-related transitions, and harvest-window timing.

<p align="center">
  <img src="imgs/growth_modelling.png" width="700">
</p>

<p align="center">Example logistic growth fitting, growth-rate analysis, and growth-acceleration analysis.</p>

---

---

## Citation

If you use this code, please cite:

```text
Ren, Y., & Argyropoulos, D. Multi-view 3D reconstruction with mushroom baseline differential segmentation for oyster mushroom growth phenotyping and harvest-window prediction.
```

---

## Acknowledgment



---
