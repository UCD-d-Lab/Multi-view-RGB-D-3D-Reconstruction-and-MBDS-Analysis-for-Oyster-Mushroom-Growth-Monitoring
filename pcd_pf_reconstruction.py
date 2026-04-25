# %% load packages and functions
import pyrealsense2 as rs
import numpy as np
import cv2
import calculate_rmsd_kabsch as rmsd
import open3d as o3d
import os
import sys
import atexit
from pathlib import Path
from collections import defaultdict
from datetime import datetime
from bag_manger import bag_frame, Transformation


# ===== Constant =====
CAPTURE_ROOT = "path/to/capture/root"

# CAMERA_LAYOUT = "5cam" or "3cam"
# For the 3-camera configuration, use top + front-left + front-right cameras.
# For the 5-camera configuration, include all five cameras.
# Path to calibration files (single-frame chessboard)
bag_files = [
    "path/to/chessboard1.bag", 
    "path/to/chessboard2.bag",
    "path/to/chessboard3.bag", 
    "path/to/chessboard4.bag", 
    "path/to/chessboard5.bag",
]

# Output path
save_path = "path/to/output/directory/Generated_PCD"  # output directory for generated PCD files (one per timestamp group)

# Calibration parameters
chessboard_height = 9  # points
chessboard_width = 6
square_size = 0.0255  # meters
TARGET_FRAME_NO = 1  # target frame number in bag for processing

# Pointcloud filtering parameters
z_min, z_max = 0.3, 0.6  # Depth limitations for pointcloud generation (m)

# ---- Color filtering params (kept consistent with Single_Processing) ----
IMG_ANALYSIS_SAVE = False  # Save HSV/Lab/RGB and channels for ImageJ analysis
SHOW_PREVIEWS = False      # Show OpenCV windows during batch

# HSV/S/Lab/RGB ranges (tune as needed)
LOWER_HSV = (5, 5, 5)
UPPER_HSV = (80, 255, 255)
LOWER_LAB = (5, 120, 15) 
UPPER_LAB = (255, 255, 255)
LOWER_RGB = (5, 5, 0)
UPPER_RGB = (255, 255, 255)

# ROI (image coords)
CropArea = [150, -50, 400, 950]  # y1, y2, x1, x2

# Reference camera
ref_serial = "serial_number_for_ref_camera"

# ---- Multi-frame controls for SAMPLE stage ----
SAMPLE_MAX_FRAMES = None   # None = use all frames in each .bag
SAMPLE_FRAME_STRIDE = 1    # >1 to subsample

# ---- Downsample & Denoise AFTER merge ----
VOXEL_SIZE = 0.001         # set None to skip
REMOVE_OUTLIERS = False
OUTLIER_METHOD = 'stat'    # 'stat' or 'radius'
NB_NEIGHBORS = 20          # for 'stat'
STD_RATIO = 2.0            # for 'stat'
RADIUS = 0.01              # for 'radius' (meters)
MIN_POINTS = 8             # for 'radius'

# pointcloud crop area:
x1, x2 = -0.16, 0.15
y1, y2 = -0.16, 0.18
z1, z2 = -0.68, -0.2
# ===== Function =====
def group_sample_bags_by_timestamp(root_dir: str):
    groups = defaultdict(list)
    root = Path(root_dir)
    if not root.exists():
        print(f"[WARN] CAPTURE_ROOT not exists: {root}")
        return {}

    for serial_dir in root.iterdir():
        if not serial_dir.is_dir():
            continue
        for day_dir in serial_dir.iterdir():
            if not day_dir.is_dir():
                continue
            for bag_file in day_dir.glob("*.bag"):
                name = bag_file.stem  # e.g. 2025-08-20_19-01-39 or Chessboard
                if name.lower() == "chessboard":
                    continue
                groups[name].append(str(bag_file))

    groups_sorted = dict(sorted(groups.items(), key=lambda kv: kv[0]))
    print(f"[Info] Indexed {sum(len(v) for v in groups_sorted.values())} bags across {len(groups_sorted)} time groups.")
    return groups_sorted


def cv_find_chessboard(infrared_frame, chessboard_params):
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    chessboard_found, corners = cv2.findChessboardCorners(
        infrared_frame, (chessboard_params[0], chessboard_params[1])
    )
    if chessboard_found:
        corners = cv2.cornerSubPix(infrared_frame, corners, (11, 11), (-1, -1), criteria)
        corners = np.transpose(corners, (2, 0, 1))
    return chessboard_found, corners


def get_depth_at_pixel(depth_frame, pixel_x, pixel_y):
    return depth_frame[int(round(pixel_y)), int(round(pixel_x))]


def get_chessboard_points_3D(chessboard_params):
    assert len(chessboard_params) == 3
    width = chessboard_params[0]
    height = chessboard_params[1]
    square_size = chessboard_params[2]
    objp = np.zeros((width * height, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    return objp.transpose() * square_size


def calculate_transformation_kabsch(src_points, dst_points):
    assert src_points.shape == dst_points.shape
    if src_points.shape[0] != 3:
        raise Exception("The input data matrix had to be transposed in order to compute transformation.")

    src_points = src_points.transpose()
    dst_points = dst_points.transpose()

    src_points_centered = src_points - rmsd.centroid(src_points)
    dst_points_centered = dst_points - rmsd.centroid(dst_points)

    rotation_matrix = rmsd.kabsch(src_points_centered, dst_points_centered)
    rmsd_value = rmsd.kabsch_rmsd(src_points_centered, dst_points_centered)

    translation_vector = rmsd.centroid(dst_points) - np.matmul(
        rmsd.centroid(src_points), rotation_matrix
    )

    return rotation_matrix.transpose(), translation_vector.transpose(), rmsd_value


def create_colored_point_cloud(depth_frame, infrared_or_color_frame, intrinsics, transformation):
    h, w = depth_frame.shape
    points = []
    colors = []

    is_rgb = (infrared_or_color_frame.ndim == 3 and infrared_or_color_frame.shape[2] == 3)
    color_norm = 255.0 if is_rgb else float(np.iinfo(infrared_or_color_frame.dtype).max)

    for y in range(h):
        for x in range(w):
            z = depth_frame[y, x]
            if z == 0:
                continue

            X = (x - intrinsics.ppx) / intrinsics.fx * z
            Y = (y - intrinsics.ppy) / intrinsics.fy * z
            pt = np.array([[X], [Y], [z]])
            pt_trans = transformation.apply_transformation(pt)

            if is_rgb:
                rgb = infrared_or_color_frame[y, x].astype(np.float32) / color_norm
                color = rgb.tolist()
            else:
                gray = float(infrared_or_color_frame[y, x]) / color_norm
                color = [gray, gray, gray]

            points.append(pt_trans.flatten())
            colors.append(color)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    return pcd


def crop_by_range(pcd: o3d.geometry.PointCloud, x_range, y_range, z_range) -> o3d.geometry.PointCloud:
    pts = np.asarray(pcd.points)
    mask = (
        (pts[:, 0] >= x_range[0]) & (pts[:, 0] <= x_range[1]) &
        (pts[:, 1] >= y_range[0]) & (pts[:, 1] <= y_range[1]) &
        (pts[:, 2] >= z_range[0]) & (pts[:, 2] <= z_range[1])
    )
    idx = np.nonzero(mask)[0]
    return pcd.select_by_index(idx)

# ---------- iterate all frames of a sample bag (aligned color->depth) ----------
def safe_iter_sample_frames(bag_path, stride=1, max_frames=None):
    try:
        for data in iter_sample_frames(bag_path, stride, max_frames):
            yield data
    except RuntimeError as e:
        print(f"[Skip] Cannot read bag {bag_path}: {e}")
        return

def iter_sample_frames(bag_path, stride=1, max_frames=None,
                       align_to="depth", preroll=0, wait_ms=5000,
                       repeat=False, max_consecutive_timeouts=3):

    pipe = rs.pipeline()
    cfg  = rs.config()

    # Single pass by default; caller may set repeat=True for a retry pass if desired
    rs.config.enable_device_from_file(cfg, bag_path, repeat)
    cfg.enable_stream(rs.stream.depth)
    cfg.enable_stream(rs.stream.color)

    profile = pipe.start(cfg)
    try:
        playback = profile.get_device().as_playback()
        playback.set_real_time(False)  # avoid first-pass frame drops

        # Choose alignment target
        align = rs.align(rs.stream.color) if align_to == "color" else rs.align(rs.stream.depth)

        dev = profile.get_device()
        serial = dev.get_info(rs.camera_info.serial_number)
        depth_scale = float(dev.first_depth_sensor().get_depth_scale())
        intrinsics  = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        # Tiny preroll is enough for ~1s bags
        for _ in range(max(0, int(preroll))):
            try:
                pipe.wait_for_frames(wait_ms)
            except RuntimeError:
                break

        i = 0
        yielded = 0
        saw_any_stream = False      # became True if either df or cf ever appeared
        consec_timeouts = 0

        while True:
            try:
                fs = pipe.wait_for_frames(wait_ms)
                consec_timeouts = 0  # got something (even None), reset the counter
            except RuntimeError as e:
                consec_timeouts += 1
                # Hard error only if we never saw any stream AND nothing yielded yet
                if not saw_any_stream and yielded == 0:
                    raise RuntimeError(f"Frame didn't arrive within {wait_ms} ms") from e
                # Already saw some frames: allow a few consecutive timeouts then stop gracefully
                if consec_timeouts >= max(1, int(max_consecutive_timeouts)):
                    break
                else:
                    # soft wait-and-continue
                    continue

            if not fs:
                i += 1
                continue

            # Align frames
            try:
                fs = align.process(fs)
            except Exception:
                i += 1
                continue

            df = fs.get_depth_frame()
            cf = fs.get_color_frame()

            if df or cf:
                saw_any_stream = True

            if not (df and cf):
                i += 1
                continue

            if i % max(1, int(stride)) != 0:
                i += 1
                continue

            depth_m = np.asanyarray(df.get_data()).astype(np.float32) * depth_scale
            color   = np.asanyarray(cf.get_data())
            yield i, serial, intrinsics, depth_m, color

            yielded += 1
            i += 1
            if (max_frames is not None) and (yielded >= max_frames):
                break
    finally:
        pipe.stop()

# %% Logger setup
# ==== Tee stdout/stderr to a readable log file ====
LOG_DIR = Path(save_path) / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

log_file_path = LOG_DIR / f"run_{datetime.now():%Y-%m-%d_%H-%M-%S}.log"

class Tee:
    def __init__(self, *streams):
        self.streams = streams
    def write(self, data):
        for s in self.streams:
            try:
                s.write(data)
                s.flush()
            except Exception:
                pass
    def flush(self):
        for s in self.streams:
            try:
                s.flush()
            except Exception:
                pass

_log_fp = open(log_file_path, "w", encoding="utf-8", buffering=1)

sys.stdout = Tee(sys.__stdout__, _log_fp)
sys.stderr = Tee(sys.__stderr__, _log_fp)
atexit.register(_log_fp.close)

print(f"[Log] Logging to: {log_file_path}")

# %% 1: Calibration (single-frame, unchanged)
chessboard_params = [chessboard_height, chessboard_width, square_size]
corners3D = {}

for bag in bag_files:
    bf_depth = bag_frame(bag, target_frame_No=TARGET_FRAME_NO)
    bf_depth.extract_depth()
    serial = bf_depth.serial_number
    depth_frame = bf_depth.frame

    bf_infrared = bag_frame(bag, target_frame_No=TARGET_FRAME_NO)
    bf_infrared.extract_infrared()
    infrared_frame = bf_infrared.frame

    found_corners, points2D = cv_find_chessboard(infrared_frame, chessboard_params)
    corners3D[serial] = [found_corners, None, None, None]

    if found_corners:
        points3D = np.zeros((3, len(points2D[0])))
        validPoints = [False] * len(points2D[0])
        for index in range(len(points2D[0])):
            corner = points2D[:, index].flatten()
            depth = get_depth_at_pixel(depth_frame, corner[0], corner[1])
            if depth != 0 and depth is not None:
                validPoints[index] = True
                X = (corner[0] - bf_infrared.intrinsics.ppx) / bf_infrared.intrinsics.fx * depth
                Y = (corner[1] - bf_infrared.intrinsics.ppy) / bf_infrared.intrinsics.fy * depth
                Z = depth
                points3D[:, index] = [X, Y, Z]
        corners3D[serial] = found_corners, points2D, points3D, validPoints

retval = {}
for (serial, [found_corners, points2D, points3D, validPoints]) in corners3D.items():
    objectpoints = get_chessboard_points_3D(chessboard_params)
    retval[serial] = [False, None, None, None]

    if found_corners is True:
        valid_object_points = objectpoints[:, validPoints]
        valid_observed_object_points = points3D[:, validPoints]

        if valid_object_points.shape[1] < 5:
            print("Not enough points have a valid depth for calculating the transformation")
        else:
            rotation_matrix, translation_vector, rmsd_value = calculate_transformation_kabsch(
                valid_object_points, valid_observed_object_points
            )
            retval[serial] = [
                True,
                Transformation(rotation_matrix, translation_vector),
                points2D,
                rmsd_value,
            ]
            print("RMS error for calibration with device number", serial, "is :", rmsd_value, "m")

transformation_files = {}
for serial in retval:
    transformation_files[serial] = retval[serial][1].inverse()  # cam -> board

# %% 2: Batch — for each timestamp group, use **all frames** of each bag
groups = group_sample_bags_by_timestamp(CAPTURE_ROOT)

save_dir = os.path.dirname(save_path)
os.makedirs(save_dir, exist_ok=True)

for ts, bag_list in groups.items():
    print(f"\n[Process] {ts} -> {len(bag_list)} bags at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    if len(bag_list) < 2:
        print(f"  [Skip] Incomplete bag group ({len(bag_list)} bags), need 2.")
        continue

    combined_pcds = []

    for bag_path in bag_list:
        first_serial = None
        for idx, serial, intrinsics, depth_frame, colored_frame in safe_iter_sample_frames(
            bag_path, stride=SAMPLE_FRAME_STRIDE, max_frames=SAMPLE_MAX_FRAMES):
            if first_serial is None:
                first_serial = serial
                if serial not in transformation_files:
                    print(f"  [Skip] serial {serial} not in calibration results")
                    break
                transform = transformation_files[serial]
                # print(f"  Use transform of cam {serial}")

            # --- Depth range filtering ---
            depth_frame_filtered = np.where((depth_frame >= z_min) & (depth_frame <= z_max), depth_frame, 0.0)
            if not np.any(depth_frame_filtered):
                continue

            # --- Color masks (as in Single_Processing) ---
            colored_bgr = cv2.cvtColor(colored_frame, cv2.COLOR_RGB2BGR)
            mask_valid_depth = (depth_frame_filtered > 0).astype(np.uint8)
            color_preview = colored_bgr.copy()
            color_preview[mask_valid_depth == 0] = (0, 0, 0)

            hsv = cv2.cvtColor(color_preview, cv2.COLOR_BGR2HSV)
            lab = cv2.cvtColor(color_preview, cv2.COLOR_BGR2Lab)
            rgb = cv2.cvtColor(color_preview, cv2.COLOR_BGR2RGB)

            mask_hsv = cv2.inRange(hsv, np.array(LOWER_HSV, dtype=np.uint8), np.array(UPPER_HSV, dtype=np.uint8))
            mask_lab = cv2.inRange(lab, np.array(LOWER_LAB, dtype=np.uint8), np.array(UPPER_LAB, dtype=np.uint8))
            mask_rgb = cv2.inRange(rgb, np.array(LOWER_RGB, dtype=np.uint8), np.array(UPPER_RGB, dtype=np.uint8))
            color_mask_orig = ((mask_hsv > 0) & (mask_lab > 0) & (mask_rgb > 0)).astype(np.uint8) * 255

            color_mask = np.zeros_like(color_mask_orig)
            color_mask[CropArea[0]:CropArea[1], CropArea[2]:CropArea[3]] = color_mask_orig[CropArea[0]:CropArea[1], CropArea[2]:CropArea[3]]

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN,  kernel, iterations=1)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel, iterations=1)



            # Apply to depth
            depth_frame_roi = np.zeros_like(depth_frame_filtered)
            depth_frame_roi[CropArea[0]:CropArea[1], CropArea[2]:CropArea[3]] = depth_frame_filtered[CropArea[0]:CropArea[1], CropArea[2]:CropArea[3]]
            depth_frame_roi[color_mask == 0] = 0

            # --- Build PCD for this frame ---
            pcd = create_colored_point_cloud(depth_frame_roi, colored_frame, intrinsics, transform)

            # board -> reference camera
            T_r_to_board = transformation_files[ref_serial]
            T_board_to_r = T_r_to_board.inverse()
            pcd.transform(T_board_to_r.pose_mat)

            # view convention (Y up, Z back negative)
            F_cam_to_view = np.eye(4)
            F_cam_to_view[1, 1] = -1
            F_cam_to_view[2, 2] = -1
            pcd.transform(F_cam_to_view)

            combined_pcds.append(pcd)

    if not combined_pcds:
        print(f"[Warn] Empty point cloud group at {ts}, skip saving.")
        continue

    # --- Merge all frames/bags for this timestamp ---
    merged_pcd = o3d.geometry.PointCloud()
    for p in combined_pcds:
        merged_pcd += p

    # --- Downsample & denoise AFTER merge ---
    if VOXEL_SIZE:
        merged_pcd = merged_pcd.voxel_down_sample(VOXEL_SIZE)

    if REMOVE_OUTLIERS and len(merged_pcd.points) > 0:
        if OUTLIER_METHOD == 'stat':
            before = np.asarray(merged_pcd.points).shape[0]
            merged_pcd, ind = merged_pcd.remove_statistical_outlier(nb_neighbors=NB_NEIGHBORS, std_ratio=STD_RATIO)
            after = np.asarray(merged_pcd.points).shape[0]
            print(f"[Denoise] SOR: {before} -> {after} (k={NB_NEIGHBORS}, std={STD_RATIO})")
        elif OUTLIER_METHOD == 'radius':
            before = np.asarray(merged_pcd.points).shape[0]
            merged_pcd, ind = merged_pcd.remove_radius_outlier(nb_points=MIN_POINTS, radius=RADIUS)
            after = np.asarray(merged_pcd.points).shape[0]
            print(f"[Denoise] ROR: {before} -> {after} (r={RADIUS}, min={MIN_POINTS})")

    # --- Optional crop in view coords ---
    pcd_crop = crop_by_range(merged_pcd, x_range=(x1, x2), y_range=(y1, y2), z_range=(z1, z2))
    merged_pcd = pcd_crop

    # --- Save per timestamp ---
    out_base = f"Generated_PCD_{ts}"
    save_dir = os.path.dirname(save_path)
    os.makedirs(save_dir, exist_ok=True)

    # ply_path = os.path.join(save_dir, f"{out_base}.ply")
    # o3d.io.write_point_cloud(ply_path, merged_pcd)
    # print(f"[Save] {ply_path}")

    pcd_path = os.path.join(save_dir, f"{out_base}.pcd")
    o3d.io.write_point_cloud(pcd_path, merged_pcd) 
    ts_suffix = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    print(f"[Save] {pcd_path} at {ts_suffix}")

print("\n[Done] All groups processed chronologically (multi-frame per bag).")
