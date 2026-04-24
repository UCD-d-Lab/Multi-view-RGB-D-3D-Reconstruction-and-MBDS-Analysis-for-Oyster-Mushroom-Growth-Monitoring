"""
Dimension and volume estimation for oyster mushroom point clouds.

This script implements the MBDS-based segmentation and voxel-based
volume estimation described in Sections 2.4 and 2.5 of the manuscript.

Inputs:
    - Baseline PCD before visible mushroom emergence
    - Time-series reconstructed PCD files

Outputs:
    - CSV file containing width, depth, height, and volume estimates
    - Optional segmented PCD files

Notes:
    - Update input/output paths or provide them through command-line arguments.
    - columns_only_cm3 corresponds to the final volume used in the research.
"""

import os, re, csv, copy
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy import ndimage as ndi
import time

# ========= Path and batch settings =========
BASELINE_PATH = "Path to Baseline PCD"# Baseline
CURRENT_DIR   = "Path to Batch Directory"  # Batch directory
OUT_CSV_PATH  = "Path to Output CSV File"
OUT_DEBUG_DIR = "Path to Debug Directory"   # Can be empty if you want to save intermediate results
OUT_PCD_DIR = "Path to Output PCD Directory"
os.makedirs(os.path.dirname(OUT_CSV_PATH), exist_ok=True)
if OUT_DEBUG_DIR: os.makedirs(OUT_DEBUG_DIR, exist_ok=True)
if OUT_PCD_DIR: os.makedirs(OUT_PCD_DIR, exist_ok=True)

# ========= Parameters (consistent with single-run script) =========
# Registration/preprocessing
DS = 0.002
ICP_VOXEL    = 0.005
ICP_MAX_CORR = 0.02
USE_ROBUST   = True

# Differential and cleanup
TAU_COARSE   = 0.025
SNN_NB       = 40
SNN_STD      = 1.5
ROR_R        = 0.008
ROR_MIN      = 10
CLUSTER_EPS  = 0.005
CLUSTER_MIN  = 30 #Original 50

# Cap construction
FOOTPRINT_GROW = 0.02

# Voxel volume
VOXEL_SIZE_VOL      = 0.001
MIN_POINTS_PER_VOX  = 1
CLOSING_ITERS       = 2
SPLAT_R             = 1
FLOOR_THICK_VOX     = 1
CAP_ONLY_FOOTPRINT  = True

#Save options
SAVE = True
TAG = time.strftime("%Y-%m-%d_%H-%M-%S")
# ========= Small utilities =========
def info(msg): print(f"[INFO] {msg}")
def warn(msg): print(f"[WARN] {msg}")

def load_pcd(path, voxel=None):
    p = o3d.io.read_point_cloud(path)
    if voxel: p = p.voxel_down_sample(voxel)
    if p.is_empty():
        raise RuntimeError(f"Empty PCD: {path}")
    return p

def save_pcd(path, pcd):
    if SAVE: o3d.io.write_point_cloud(path, pcd); info(f"PCD saved: {path}")
def orient_normals(pcd, radius=0.02, max_nn=30):
    if len(pcd.points) == 0: return pcd
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    c = pcd.get_center()
    pcd.orient_normals_towards_camera_location(c + np.array([0,0,1.0]))
    return pcd

def _robust_kernel(c=1.0):
    try:
        RK = o3d.pipelines.registration.RobustKernelType
        return o3d.pipelines.registration.RobustKernel(RK.Tukey, c)
    except Exception:
        try:
            return o3d.pipelines.registration.TukeyLoss(c)
        except Exception:
            return None

def register_icp(src, tgt, voxel=ICP_VOXEL, max_corr=ICP_MAX_CORR, use_robust=USE_ROBUST, max_iter=60):
    s = src.voxel_down_sample(voxel); t = tgt.voxel_down_sample(voxel)
    orient_normals(s); orient_normals(t)
    init = np.eye(4)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        s, t, max_corr, init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter//3)
    )
    if use_robust:
        loss = _robust_kernel(1.0)
        try:
            est = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        except TypeError:
            est = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss=loss)
    else:
        est = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    reg = o3d.pipelines.registration.registration_icp(
        s, t, max_corr, reg_p2p.transformation, est,
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
    )
    T = reg.transformation
    out = copy.deepcopy(src); out.transform(T)
    return out, T, reg.fitness, reg.inlier_rmse

def subtract_pointclouds(A, B, tau):
    """Returns the point set of B-A (points in B that are farther than tau from A)"""
    A_np = np.asarray(A.points); B_np = np.asarray(B.points)
    cols = np.asarray(B.colors) if B.has_colors() else None
    if len(A_np)==0 or len(B_np)==0:
        out = o3d.geometry.PointCloud()
        out.points = o3d.utility.Vector3dVector(B_np.copy())
        if cols is not None:
            out.colors = o3d.utility.Vector3dVector(cols.copy())
        return out
    tree = cKDTree(A_np); dist,_ = tree.query(B_np, k=1)
    keep = np.where(dist > tau)[0]
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(B_np[keep])
    if cols is not None: out.colors = o3d.utility.Vector3dVector(cols[keep])
    return out

def denoise_pcd(pcd, sor_nb=SNN_NB, sor_std=SNN_STD, ror_radius=ROR_R, ror_min=ROR_MIN):
    if len(pcd.points)==0: return pcd
    p1,_ = pcd.remove_statistical_outlier(nb_neighbors=sor_nb, std_ratio=sor_std)
    if len(p1.points)==0: return pcd
    p2,_ = p1.remove_radius_outlier(nb_points=ror_min, radius=ror_radius)
    return p2 if len(p2.points)>0 else p1

# def keep_largest_cluster(pcd, eps=CLUSTER_EPS, min_points=CLUSTER_MIN):
#     if len(pcd.points)==0: return pcd
#     labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
#     if labels.size==0 or labels.max()<0: return pcd
#     keep = int(np.bincount(labels[labels>=0]).argmax())
#     idx = np.where(labels==keep)[0]
#     return pcd.select_by_index(idx)
def keep_largest_cluster(pcd, eps=CLUSTER_EPS, min_points=CLUSTER_MIN):
    n = len(pcd.points)
    if n == 0:
        return pcd

    if n < min_points:
        return o3d.geometry.PointCloud()

    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))

    if labels.size == 0 or np.all(labels < 0):
        return o3d.geometry.PointCloud()

    counts = np.bincount(labels[labels >= 0])
    keep = int(counts.argmax())
    idx = np.where(labels == keep)[0]

    if idx.size < min_points:
        return o3d.geometry.PointCloud()

    return pcd.select_by_index(idx)

def build_cap_from_baseline(mushroom, baseline, grow_r=FOOTPRINT_GROW):
    """Grows a cap (base) from the neighborhood of the mushroom projection in the baseline point cloud"""
    M = np.asarray(mushroom.points)
    B = np.asarray(baseline.points)
    cap = o3d.geometry.PointCloud()
    if M.size==0 or B.size==0:
        return cap
    tree = cKDTree(B)
    _, nn = tree.query(M, k=1)
    seed = np.unique(nn)
    mask = np.zeros(len(B), bool)
    for i in seed:
        nbr = tree.query_ball_point(B[i], r=grow_r)
        mask[nbr] = True
    cap.points = o3d.utility.Vector3dVector(B[mask])
    if baseline.has_colors():
        cap.colors = o3d.utility.Vector3dVector(np.asarray(baseline.colors)[mask])
    return cap

# ========= Voxel volume =========
def voxel_volume_with_floor(mushroom, cap,
                            VOX=VOXEL_SIZE_VOL, min_pts=MIN_POINTS_PER_VOX,
                            CLOSE_ITERS=CLOSING_ITERS, splat_r=SPLAT_R,
                            FLOOR_THK=FLOOR_THICK_VOX, ONLY_FP=CAP_ONLY_FOOTPRINT):

    Pm = np.asarray(mushroom.points) if len(mushroom.points) else np.empty((0,3))
    Pc = np.asarray(cap.points)      if len(cap.points)      else np.empty((0,3))

    if Pm.size == 0:
        return {'solid_cm3':0.0,'floor_cm3':0.0,'columns_only_cm3':0.0,'total_cm3':0.0,
                'extent_m':np.array([0.0,0.0,0.0]),'bbox_extent_m': np.array([0.0, 0.0, 0.0])}

    # --- Build a voxel grid  ---
    mn = Pm.min(axis=0); mx = Pm.max(axis=0)
    span = np.maximum(mx - mn, VOX)
    dims = np.ceil(span / VOX).astype(int)
    pad  = 10
    origin = mn - pad * VOX
    shape = tuple((dims + 2*pad).tolist())  # (X,Y,Z)

    # AABB (meters)
    # Here, the voxel grid is used to align with the Voxel volume standard.
    mins = origin
    maxs = origin + VOX * (np.array(shape) - 1)
    extent = maxs - mins  # (dx, dy, dz)

    # --- Mushroom voxel shell -> solid ---
    idx_m = np.floor((Pm - origin) / VOX).astype(int) + pad
    idx_m = np.clip(idx_m, 0, np.array(shape) - 1)
    Gm = np.zeros(shape, np.int32)
    for x,y,z in idx_m: Gm[x,y,z] += 1
    shell_m = (Gm >= min_pts)
    if splat_r > 0:
        st = np.ones((1*splat_r+1,)*3, np.uint8)
        shell_m = ndi.binary_dilation(shell_m, structure=st)
    if CLOSE_ITERS > 0:
        shell_m = ndi.binary_closing(shell_m, structure=ndi.generate_binary_structure(3,2),
                                     iterations=CLOSE_ITERS)
    solid_m = ndi.binary_fill_holes(shell_m)

    # --- bbox from CLOSED solid (after closing) ---
    occ = np.argwhere(solid_m)  # N x 3 voxel indices
    if occ.size == 0:
        bbox_extent_m = np.array([0.0, 0.0, 0.0])
    else:
        mn_idx = occ.min(axis=0)
        mx_idx = occ.max(axis=0)
        bbox_extent_m = (mx_idx - mn_idx + 1) * VOX  # meters

    # --- cap → floor calculated from footprint ---
    occ_c = np.zeros(shape, bool)
    if Pc.size:
        idx_c = np.floor((Pc - origin) / VOX).astype(int) + pad
        idx_c = np.clip(idx_c, 0, np.array(shape) - 1)
        for x,y,z in idx_c: occ_c[x,y,z] = True
        if splat_r > 0:
            st = np.ones((1*splat_r+1,)*3, np.uint8)
            occ_c = ndi.binary_dilation(occ_c, structure=st)


    if ONLY_FP:
        allowed_xz = np.any(solid_m, axis=1)  # shape: (X,Z)
    else:
        allowed_xz = np.any(occ_c, axis=1)    # shape: (X,Z)

    xs, zs = np.where(allowed_xz)             

    # construct floor mask
    floor = np.zeros_like(occ_c, dtype=bool)
    for x, z in zip(xs, zs):
        col_cap = occ_c[x, :, z]
        if not np.any(col_cap):
            continue
        yf = int(np.where(col_cap)[0].min())
        y0 = max(0, yf - FLOOR_THK)
        y1 = min(occ_c.shape[1] - 1, yf + FLOOR_THK)
        floor[x, y0:y1+1, z] = True

    # column filling
    filled = solid_m.copy()
    for x, z in zip(xs, zs):
        col_floor = floor[x, :, z]
        col_body  = solid_m[x, :, z]
        if not (np.any(col_floor) and np.any(col_body)):
            continue
        y_floor = int(np.where(col_floor)[0].min())
        y_top   = int(np.where(col_body)[0].max())
        if y_top > y_floor + 2:
            filled[x, y_floor+2:y_top-2, z] = True

    # return volume (cm3)
    vox_cm3 = (VOX * 100.0) ** 3  # m -> cm
    solid_only    = solid_m & ~floor
    floor_only    = floor & ~solid_m
    columns_only  = filled & ~(solid_m | floor)   
    total_mask    = solid_only | floor_only | columns_only  

    solid_cm3        = vox_cm3 * int(np.count_nonzero(solid_only))
    floor_cm3        = vox_cm3 * int(np.count_nonzero(floor_only))
    columns_only_cm3 = vox_cm3 * int(np.count_nonzero(columns_only))
    total_cm3        = vox_cm3 * int(np.count_nonzero(total_mask))

    return {
        'solid_cm3': float(solid_cm3),
        'floor_cm3': float(floor_cm3),
        'columns_only_cm3': float(columns_only_cm3),
        'total_cm3': float(total_cm3),
        'extent_m': extent,
        'bbox_extent_m': bbox_extent_m 
    }
# ========= main pipline for each PCD =========
def compute_volume_for_pair(baseline_path, current_path, save_debug=False):
    base = load_pcd(baseline_path)
    curr_1 = load_pcd(current_path)
    # Downsample for speed
    base = base.voxel_down_sample(DS)
    curr = curr_1.voxel_down_sample(DS)
    #base = keep_largest_cluster(denoise_pcd(base))
    curr = keep_largest_cluster(curr_1)

    # 1) Coarse difference to get "current bag candidate" (helps ICP)
    M0 = subtract_pointclouds(base, curr, 0.005)   # curr - base
    P  = np.asarray(curr.points)
    Q  = np.asarray(M0.points)
    if Q.size:
        tree = cKDTree(Q)
        lists = tree.query_ball_point(P, r=0.008)
        mask = np.array([len(l)==0 for l in lists], bool)
        bag_only = o3d.geometry.PointCloud()
        bag_only.points = o3d.utility.Vector3dVector(P[mask])
        if curr.has_colors():
            bag_only.colors = o3d.utility.Vector3dVector(np.asarray(curr.colors)[mask])
    else:
        bag_only = curr

    # 2) ICP（bag_only → baseline）
    bag_aligned, T, fit, rmse = register_icp(bag_only, base)

    # 3) Align the entire current point cloud
    curr_aligned = copy.deepcopy(curr)
    curr_aligned.transform(T)

    # 4) Post-alignment difference: Mushroom body & base candidate
    mushroom = subtract_pointclouds(base, curr_aligned, TAU_COARSE)  # curr_aligned - base
    bag      = subtract_pointclouds(curr_aligned, base, TAU_COARSE)  # base - curr_aligned

    # Cleanup and largest cluster
    # mushroom = keep_largest_cluster(denoise_pcd(mushroom))
    # bag      = keep_largest_cluster(denoise_pcd(bag))
    mushroom = keep_largest_cluster(mushroom)
    bag      = keep_largest_cluster(bag)
    # 5) Grow cap from baseline
    cap = build_cap_from_baseline(mushroom, base, grow_r=FOOTPRINT_GROW)

    # 6) Voxel volume + AABB breakdown
    br = voxel_volume_with_floor(mushroom, cap)
    ext = br.get("bbox_extent_m", [float("nan"), float("nan"), float("nan")])
    # Optional: Save intermediate results
    if save_debug and OUT_DEBUG_DIR:
        stem = os.path.splitext(os.path.basename(current_path))[0]
        o3d.io.write_point_cloud(os.path.join(OUT_DEBUG_DIR, f"{stem}_mushroom.pcd"), mushroom)
        o3d.io.write_point_cloud(os.path.join(OUT_DEBUG_DIR, f"{stem}_cap.pcd"),       cap)



    return mushroom, curr_1, curr, {
        "fitness": float(fit),
        "rmse": float(rmse),
        "bbox_dx_mm": float(ext[0]) * 1000.0,
        "bbox_dy_mm": float(ext[1]) * 1000.0,
        "bbox_dz_mm": float(ext[2]) * 1000.0,
        "solid_cm3": br["solid_cm3"],
        "floor_cm3": br["floor_cm3"],
        "columns_only_cm3": br["columns_only_cm3"],
        "total_cm3": br["total_cm3"],
        "volume_mL": br["total_cm3"],
    }


# ========= tools
def is_generated_pcd(name: str) -> bool:
    return name.lower().startswith("generated_pcd_") and name.lower().endswith(".pcd")

def parse_timestamp_from_name(name: str) -> str:
    """
    Extract 'YYYY-MM-DD HH-MM-SS' from 'Generated_PCD_YYYY-MM-DD_HH-MM-SS.pcd'.
    If no match, return an empty string.
    """
    m = re.search(r"Generated_PCD_(\\d{4}-\\d{2}-\\d{2})_(\\d{2}-\\d{2}-\\d{2})\\.pcd", name, re.I)
    return f"{m.group(1)} {m.group(2)}" if m else ""

# ========= bacth processing main program =========
def main():
    baseline = BASELINE_PATH
    if not os.path.isfile(baseline):
        raise FileNotFoundError(baseline)

    # Collect PCD files in the directory
    files = [os.path.join(CURRENT_DIR, f) for f in os.listdir(CURRENT_DIR) if is_generated_pcd(f)]
    files.sort()

    # Filter out the one identical to baseline (if in the same directory)
    files = [f for f in files if os.path.abspath(f) != os.path.abspath(baseline)]
    if not files:
        warn("No point clouds found for batch processing.")
        return

    info(f"Baseline: {baseline}")
    info(f"Total {len(files)} point clouds to process.")

    # Output CSV
    header = [
        "file", "timestamp",
        "fitness", "rmse",
        "bbox_dx_mm", "bbox_dy_mm", "bbox_dz_mm",
        "solid_cm3", "floor_cm3", "columns_only_cm3", "total_cm3", "volume_mL"
    ]
    with open(OUT_CSV_PATH, "w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(header)

        for i, cur in enumerate(files, 1):
            info(f"[{i}/{len(files)}] Processing: {cur}")
            try:
                Mushroom, curr_1, curr, res = compute_volume_for_pair(baseline, cur, save_debug=False)
                if SAVE:
                    save_pcd(os.path.join(OUT_PCD_DIR,f"mushroom_only_{i}.pcd"),Mushroom)
                    save_pcd(os.path.join(OUT_PCD_DIR,f"Current_before_{i}.pcd"),curr_1)
                    save_pcd(os.path.join(OUT_PCD_DIR,f"Current_denoised_{i}.pcd"),curr)
                row = [
                    os.path.basename(cur),
                    parse_timestamp_from_name(os.path.basename(cur)),
                    f"{res['fitness']:.4f}",
                    f"{res['rmse']:.4f}",
                    f"{res['bbox_dx_mm']:.2f}",
                    f"{res['bbox_dy_mm']:.2f}",
                    f"{res['bbox_dz_mm']:.2f}",
                    f"{res['solid_cm3']:.2f}",
                    f"{res['floor_cm3']:.2f}",
                    f"{res['columns_only_cm3']:.2f}",
                    f"{res['total_cm3']:.2f}",
                    f"{res['volume_mL']:.2f}",
                ]
                w.writerow(row)
            except Exception as e:
                warn(f"Processing failed: {cur} -> {e}")
                row = [
                    os.path.basename(cur),
                    parse_timestamp_from_name(os.path.basename(cur)),
                    "NA","NA","NA","NA","NA","NA","NA","NA","NA","NA"
                ]
                w.writerow(row)

    info(f"Completed. Results written to {OUT_CSV_PATH}")

if __name__ == "__main__":
    main()
