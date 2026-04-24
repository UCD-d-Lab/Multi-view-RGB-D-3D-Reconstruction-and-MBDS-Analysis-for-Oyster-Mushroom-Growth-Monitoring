#%% =============== Load Packages ===============
import os
import time
import json
import signal
import threading
import pyrealsense2 as rs
from datetime import datetime, timedelta

# =============== Load Constants =================
OUTPUT_DIR = "Path to Output Directory"         # Output address
SETTINGS_JSON_BY_MODEL = {
    'D405':  "Path to Camera Setting JSON for D405", 
    'D435i': "Path to Camera Setting JSON for D435i",}
ROI_JSON = "Path to ROI Configuration JSON"

DURATION_SEC = 1.0           # Recording time for each bag file (s)
INTERVAL_MIN = 15            # Interval time between each two bag file (min)
FPS = 'auto'                 # "auto":5 for D405,6 for D435i
WIDTH, HEIGHT = 1280,720     # 1280, 720/ 848, 480    # color/depth/ir should have same resolution
WARMUP_FRAMES = 30           # frame number for warm up (high frames could lead to program crash)
stop_flag = False            # Stop Signal (should be "false" at first time)
sequential_mode = True       # True: cameras capturing one by one; False: cameras capturing at same time (need higher bandwidth)
max_retries = 5              # retry times for each camera if the capturing fail



# =================== Load Function ===================
def handle_sigint(sig, frame):
    global stop_flag
    stop_flag = True
    print("\n[Info] Recieved Stopping Signal, Conducting Stopping…")
signal.signal(signal.SIGINT, handle_sigint)

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def list_connected_serials():
    """Return a list of connected RealSense device serial numbers."""
    ctx = rs.context()
    devices = []
    for d in ctx.query_devices():
        try:
            serial = d.get_info(rs.camera_info.serial_number)
            devices.append(serial)
        except Exception:
            pass
    return devices

def get_device_model_for_serial(serial: str) -> str:
    ctx = rs.context()
    for d in ctx.query_devices():
        try:
            if d.get_info(rs.camera_info.serial_number) == serial:
                try:
                    return d.get_info(rs.camera_info.name)  
                except Exception:
                    pid = d.get_info(rs.camera_info.product_id) if d.supports(rs.camera_info.product_id) else ""
                    pl  = d.get_info(rs.camera_info.product_line) if d.supports(rs.camera_info.product_line) else ""
                    return f"{pl}-{pid}"
        except Exception:
            pass
    return ""

def resolve_fps(model: str, fps_cfg) -> int:
    if isinstance(fps_cfg, str) and fps_cfg.lower() == "auto":
        mu = (model or "").upper()
        if "D405" in mu:
            return 5
        if "D435I" in mu:
            return 6
        return 15
    return int(fps_cfg)

def pick_settings_json_for_serial(serial: str) -> str | None:
    #if serial in SETTINGS_JSON_BY_SERIAL:
    #    return SETTINGS_JSON_BY_SERIAL[serial]

    model = get_device_model_for_serial(serial) 
    model_upper = model.upper()
    for key, path in SETTINGS_JSON_BY_MODEL.items():
        if key.upper() in model_upper:
            return path
    return

def build_bag_path(base_dir: str, serial: str, ts: datetime):
    """OUTPUT_DIR/<serial>/<YYYY-MM-DD>/<YYYY-MM-DD_HHMMSS>.bag"""
    day_dir = os.path.join(base_dir, serial, ts.strftime("%Y-%m-%d"))
    ensure_dir(day_dir)
    bag_name = ts.strftime("%Y-%m-%d_%H-%M-%S") + ".bag"
    return os.path.join(day_dir, bag_name)

def load_settings_json_for_serial(serial: str, path_to_settings_file: str):

    if not os.path.isfile(path_to_settings_file):
        raise FileNotFoundError(path_to_settings_file)

    with open(path_to_settings_file, "r", encoding="utf-8") as f:
        json_text = f.read().strip()

    ctx = rs.context()
    target = None
    for d in ctx.query_devices():
        try:
            if d.get_info(rs.camera_info.serial_number) == serial:
                target = d
                break
        except Exception:
            pass

    if target is None:
        raise RuntimeError(f"Device {serial} not found")

    try:
        product_line = target.get_info(rs.camera_info.product_line)
    except Exception:
        product_line = ""

    if product_line != "D400":
        print(f"[Warn:{serial}] Not a D400 device, skip loading JSON.")
        return

    adv = rs.rs400_advanced_mode(target)

    if not adv.is_enabled():
        print(f"[Info:{serial}] Enabling Advanced Mode…")
        adv.toggle_advanced_mode(True)
        time.sleep(5.0)
        ctx = rs.context()
        target = None
        for d in ctx.query_devices():
            try:
                if d.get_info(rs.camera_info.serial_number) == serial:
                    target = d
                    break
            except Exception:
                pass
        if target is None:
            raise RuntimeError(f"[{serial}] Device disappeared after enabling Advanced Mode.")
        adv = rs.rs400_advanced_mode(target)

    adv.load_json(json_text)
    print(f"[OK:{serial}] Loaded settings JSON.")
    
def set_auto_exposure_roi_for_serial(serial: str, roi_cfg: dict, profile: rs.pipeline_profile):
    dev = profile.get_device()
    if dev.get_info(rs.camera_info.serial_number) != serial:
        return

    target = (roi_cfg.get("target") or "depth").lower()

    # 1) aquire the resolution
    want_stream = rs.stream.depth if target in ("depth", "ir") else rs.stream.color
    w = h = None
    for sp in profile.get_streams():
        if sp.stream_type() == want_stream:
            vsp = sp.as_video_stream_profile()
            w, h = vsp.width(), vsp.height()
            break

    # 2) find the target sensor
    sensor = None
    for s in dev.query_sensors():
        name = s.get_info(rs.camera_info.name).lower()
        if target == "color" and "rgb" in name:
            sensor = s; break
        if target in ("depth", "ir") and ("stereo" in name or "depth" in name):
            sensor = s; break
    if sensor is None:
        print(f"[Warn:{serial}] Can not find {target} sensor, pass")
        return

    # 3) start auto expourse
    try:
        if sensor.supports(rs.option.enable_auto_exposure):
            sensor.set_option(rs.option.enable_auto_exposure, 1)
    except Exception as e:
        print(f"[Warn:{serial}] Can not enable AE{e}")

    # 4) load ROI
    xmin = int(roi_cfg["xmin"]); ymin = int(roi_cfg["ymin"])
    xmax = int(roi_cfg["xmax"]); ymax = int(roi_cfg["ymax"])
    if w is not None and h is not None:
        xmin = max(0, min(xmin, w-1))
        xmax = max(0, min(xmax, w-1))
        ymin = max(0, min(ymin, h-1))
        ymax = max(0, min(ymax, h-1))
        xmax = max(xmax, xmin + 1)
        ymax = max(ymax, ymin + 1)

    try:
        roi = rs.region_of_interest()
        roi.min_x = xmin
        roi.min_y = ymin
        roi.max_x = xmax
        roi.max_y = ymax

        roi_sensor = sensor.as_roi_sensor()
        roi_sensor.set_region_of_interest(roi)
        print(f"[OK:{serial}] AE ROI -> ({xmin},{ymin})-({xmax},{ymax}) on {target}")
    except Exception as e:
        print(f"[Warn:{serial}] Fail Load ROI:{e}")


def record_once_for_serial(serial: str, bag_path: str):
    """Record one .bag for one device (no IMU), with warmup."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    model = get_device_model_for_serial(serial)
    fps_to_use = resolve_fps(model, FPS)

    # Enable streams (no IMU)
    config.enable_stream(rs.stream.color,   WIDTH, HEIGHT, rs.format.rgb8, fps_to_use)
    config.enable_stream(rs.stream.depth,   WIDTH, HEIGHT, rs.format.z16,  fps_to_use)
    config.enable_stream(rs.stream.infrared, 1, WIDTH, HEIGHT, rs.format.y8, fps_to_use)
    #config.enable_stream(rs.stream.infrared, 2, WIDTH, HEIGHT, rs.format.y8, fps_to_use)
    rec = None
    try:
        profile = pipeline.start(config)

        if serial in roi_map:
            set_auto_exposure_roi_for_serial(serial, roi_map[serial], profile)

            for _ in range(WARMUP_FRAMES):
                pipeline.wait_for_frames(timeout_ms = 15000)
            print(f"[Record:{serial}] Warmup done, start recording -> {bag_path}")

            dev = profile.get_device()
            rec = rs.recorder(bag_path,dev)

            t0 = time.time()
            while (time.time() - t0) < DURATION_SEC and not stop_flag:
                # Use non-blocking poll if you want tighter loop, but wait_for_frames is fine here.
                pipeline.wait_for_frames(timeout_ms=15000)
            
    except Exception as e:
        print(f"[Error:{serial}] {e}")

    finally:
        try:
            if rec is not None:
                del rec
        except Exception: 
            pass
        try:
            pipeline.stop()
            print(f"[Record:{serial}] Finished (cost {time.time()-t0:.2f}s)")
            time.sleep(5)
        except Exception:
            pass

def run_one_round(serials, ts_for_names: datetime):

    verify_timeout_ms = 3000

    if sequential_mode:
        for serial in serials:
            bag_path = build_bag_path(OUTPUT_DIR, serial, ts_for_names)
            print(f"[Info] Start recording for {serial} -> {bag_path}")

            ok = False
            for attempt in range(1, max_retries + 1):
                record_once_for_serial(serial, bag_path)
                ok = bag_has_color_and_depth(bag_path, timeout_ms=verify_timeout_ms)
                if ok:
                    print(f"[Info] Verified bag for {serial} on attempt {attempt}")
                    break

                print(f"[Warn] Verify failed for {serial} (attempt {attempt}). Will retry.")

                try:
                    if os.path.isfile(bag_path):
                        os.remove(bag_path)
                except Exception as e:
                    print(f"[Warn] Remove failed bag error: {e}")
                
                time.sleep(6) 

            if not ok:
                print(f"[Error] Giving up after retries for {serial}: {bag_path}")

            print(f"[Info] Finished recording for {serial}")


        print("[Info] Round finished for all cameras as Sequential mode.")

    else:
        def worker(serial: str):
            bag_path = build_bag_path(OUTPUT_DIR, serial, ts_for_names)
            print(f"[Info] Thread start for {serial} -> {bag_path}")

            ok = False
            for attempt in range(1, max_retries + 2):
                record_once_for_serial(serial, bag_path)
                ok = bag_has_color_and_depth(bag_path, timeout_ms=verify_timeout_ms)
                if ok:
                    print(f"[Info] Verified bag for {serial} on attempt {attempt}")
                    break

                print(f"[Warn] Verify failed for {serial} (attempt {attempt}). Will retry.")
                try:
                    if os.path.isfile(bag_path):
                        os.remove(bag_path)
                except Exception as e:
                    print(f"[Warn] Remove failed bag error: {e}")
                time.sleep(2)

            if not ok:
                print(f"[Error] Giving up after retries for {serial}: {bag_path}")

            print(f"[Info] Thread finished for {serial}")

        threads = []
        for serial in serials:
            t = threading.Thread(target=worker, args=(serial,), daemon=True)
            t.start()
            threads.append(t)

        for t in threads:
            t.join()

        time.sleep(6)
        print("[Info] Round finished for all cameras as Parallel mode.")

def bag_has_color_and_depth(bag_path: str, timeout_ms: int = 3000) -> bool:
    if not os.path.isfile(bag_path):
        print(f"[Check] Bag not found: {bag_path}")
        return False

    pipe = rs.pipeline()
    cfg = rs.config()
    try:
        cfg.enable_device_from_file(bag_path, repeat_playback=False)
        profile = pipe.start(cfg)
        playback = profile.get_device().as_playback()
        playback.set_real_time(False)

        seen_color, seen_depth = False, False

        while True:
            try:
                fs = pipe.wait_for_frames(timeout_ms)
            except Exception:
                break

            if fs:
                if fs.get_depth_frame():
                    seen_depth = True
                if fs.get_color_frame():
                    seen_color = True

                if seen_color and seen_depth:
                    break

        try:
            pipe.stop()
        except Exception:
            pass

        if not seen_color or not seen_depth:
            print(f"[Check] Missing streams: color={seen_color}, depth={seen_depth} -> {bag_path}")
        else:
            print(f"[Check] OK (color+depth): {bag_path}")
        return seen_color and seen_depth

    except Exception as e:
        print(f"[Check] Failed to open/play bag: {bag_path} ({e})")
        try:
            pipe.stop()
        except Exception:
            pass
        return False
    

#%% ============= Load Configuration Files =================
# Discover devices & Loading Camera Settings JSON if available
print("[Info] Discovering connected devices...")
serials = list_connected_serials()
if not serials:
    print("[Error] No RealSense device found. Please check connection/permissions.")
print(f"[Info] Found devices: {serials}")

for s in serials:
    chosen = pick_settings_json_for_serial(s)
    if not chosen:
        print(f"[Info:{s}] No JSON mapping found, skip loading.")
        continue
    try:
        load_settings_json_for_serial(s, chosen)
    except Exception as e:
        print(f"[Warn:{s}] Failed to load JSON '{chosen}': {e}")

try:
    with open(ROI_JSON,"r",encoding="utf-8") as f:
        roi_map = json.load(f)
except FileNotFoundError:
    roi_map = {}
    print("[Info] roi_config.json not found, skip AE-ROI.")

#%%============= Recording =================
# Start recording
print(f"[Info] Start automatic recording every {INTERVAL_MIN} min(s), each {DURATION_SEC}s. "
        f"Streams: Color/Depth/IR1/IR2 @ {WIDTH}x{HEIGHT}@ FPS:{FPS} (no IMU)")

next_time = datetime.now()  # Start immediately

while not stop_flag:
        now = datetime.now()
        if now >= next_time:
            ts_for_names = now
            print(f"[Info] === Start this turn recording @ {ts_for_names.strftime('%Y-%m-%d %H:%M:%S')} ===")

            #Recording:
            try:
                round_paths = run_one_round(serials, ts_for_names)
            except Exception as e:
                print(f"[Error] This turn recording failed: {e}")
                round_paths = []

            # plan for next turn recording
            next_time = ts_for_names + timedelta(minutes=INTERVAL_MIN)
            print(f"[Info] The time for next turn recording is {next_time.strftime('%Y-%m-%d %H:%M:%S')}")

        time.sleep(0.5) 


print("[Info] Exited Safely.")
# %%
