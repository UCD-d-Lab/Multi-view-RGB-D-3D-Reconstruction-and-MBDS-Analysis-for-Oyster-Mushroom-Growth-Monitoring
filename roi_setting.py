import argparse, json, os, sys
from pathlib import Path
import pyrealsense2 as rs
import cv2
import numpy as np

# ===========================================================
OUTFILE_DEFAULT = "path/to/output/directory"  # Default output path for ROI config JSON (can be a directory or a file path)
Resolution = [1280, 720]
# ===========================================================

def list_devices(ctx):
    devs = []
    for d in ctx.query_devices():
        try:
            sn = d.get_info(rs.camera_info.serial_number)
            devs.append(sn)
        except Exception:
            continue
    return devs

def start_pipeline_for(serial, target):
    pipe = rs.pipeline()
    cfg = rs.config()
    if serial:
        cfg.enable_device(serial)

    cfg.enable_stream(rs.stream.depth, Resolution[0], Resolution[1], rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, Resolution[0], Resolution[1], rs.format.bgr8, 30)

    profile = pipe.start(cfg)

    stream = rs.stream.depth if target == "depth" else rs.stream.color
    active = None
    for s in profile.get_streams():
        if s.stream_type() == stream:
            active = s.as_video_stream_profile()
            break
    if active is None:
        raise RuntimeError(f"Active stream for {target} not found.")
    return pipe, profile, active.width(), active.height()

class DragRect:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.drawing = False
        self.x0 = self.y0 = self.x1 = self.y1 = 0
        self.has_rect = False

    def clamp(self):
        self.x0 = max(0, min(self.x0, self.w-1))
        self.x1 = max(0, min(self.x1, self.w-1))
        self.y0 = max(0, min(self.y0, self.h-1))
        self.y1 = max(0, min(self.y1, self.h-1))

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.x0 = self.x1 = x
            self.y0 = self.y1 = y
            self.has_rect = False
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.x1, self.y1 = x, y
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.x1, self.y1 = x, y
            self.clamp()
            if abs(self.x1 - self.x0) >= 2 and abs(self.y1 - self.y0) >= 2:
                self.has_rect = True

    def get_xyxy(self):
        x_min = min(self.x0, self.x1)
        y_min = min(self.y0, self.y1)
        x_max = max(self.x0, self.x1)
        y_max = max(self.y0, self.y1)
        return x_min, y_min, x_max, y_max

def draw_overlay(img, rect: DragRect, target, wh_text):
    if rect.has_rect or rect.drawing:
        x0,y0,x1,y1 = rect.get_xyxy()
        cv2.rectangle(img, (x0,y0), (x1,y1), (255,255,255), 1)
    cv2.putText(img, f"Target: {target}", (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(img, wh_text, (8,40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(img, "Drag ROI. ENTER=accept  R=reset  Q/Esc=quit", (8, img.shape[0]-8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

def normalize_outfile_path(outfile: str|Path) -> Path:
    p = Path(outfile).expanduser()
    if p.suffix.lower() != ".json":
        p = p / "roi_config.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    return p

def pick_roi_for_device(serial, target, outfile):
    out_path = normalize_outfile_path(outfile)
    print(f"\n=== Device {serial} | target={target} ===")
    pipe, profile, w, h = start_pipeline_for(serial, target)
    name = profile.get_device().get_info(rs.camera_info.name)
    print(f"Started {name} ({serial}) at {w}x{h} for {target}")

    rect = DragRect(w, h)
    win = f"AE ROI Picker ({serial} | {target})"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, w, h)
    cv2.setMouseCallback(win, rect.on_mouse)

    accepted = None
    try:
        while True:
            frames = pipe.wait_for_frames()
            if target == "depth":
                depth = frames.get_depth_frame()
                if not depth: continue
                depth_np = np.asanyarray(depth.get_data())
                depth_8u = cv2.convertScaleAbs(depth_np, alpha=0.03) 
                vis = cv2.cvtColor(depth_8u, cv2.COLOR_GRAY2BGR)
            else:
                color = frames.get_color_frame()
                if not color: continue
                vis = np.asanyarray(color.get_data()) 

            draw_overlay(vis, rect, target, f"Resolution: {w}x{h}")
            cv2.imshow(win, vis)
            k = cv2.waitKey(1) & 0xFF
            if k in (13, 10):   # ENTER
                if rect.has_rect:
                    accepted = rect.get_xyxy()
                    break
            elif k in (ord('r'), ord('R')):
                rect = DragRect(w, h)
                cv2.setMouseCallback(win, rect.on_mouse)
            elif k in (27, ord('q'), ord('Q')):  # ESC or Q
                break
    finally:
        pipe.stop()
        cv2.destroyWindow(win)

    if accepted is None:
        print(f"[Skip] No ROI saved for {serial}:{target}")
        return None

    xmin, ymin, xmax, ymax = accepted
    print(f"[OK] ROI for {serial}:{target} -> ({xmin},{ymin})-({xmax},{ymax})")

    data = {}
    if out_path.exists():
        try:
            with out_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            data = {}

    data[serial] = {"xmin": int(xmin), "ymin": int(ymin),
                    "xmax": int(xmax), "ymax": int(ymax),
                    "target": target}
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"[Saved] {out_path}")
    return (xmin, ymin, xmax, ymax)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", help="Device serial to use. If omitted, iterate all devices.", default=None)
    ap.add_argument("--target", help="Which sensor to set AE ROI for: depth|color",
                    default="depth", choices=["depth", "color"])
 
    ap.add_argument("--outfile", help="Path to JSON file or directory for saving ROI(s).",
                    default=OUTFILE_DEFAULT)
    args = ap.parse_args()

    ctx = rs.context()
    devs = list_devices(ctx)
    if not devs:
        print("No RealSense devices found.")
        sys.exit(1)

    serials = [args.serial] if args.serial else devs
    for sn in serials:
        pick_roi_for_device(sn, args.target, args.outfile)

if __name__ == "__main__":
    main()
