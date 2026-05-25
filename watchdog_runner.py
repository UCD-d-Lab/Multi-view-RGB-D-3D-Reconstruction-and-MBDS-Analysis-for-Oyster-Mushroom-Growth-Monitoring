# watchdog_runner.py
import subprocess, sys, time, os, signal, pathlib, datetime, shutil, textwrap

# ======== configuration  ========
PYTHON_EXE = sys.executable                   # or with fix path like r"C:\Python311\python.exe"
TARGET_SCRIPT = r" # your capture script filename/absolute path"   # your capture script filename/absolute path
WORK_DIR = pathlib.Path(r"# target script directory") #pathlib.Path(__file__).parent      # target script directory
LOG_DIR = WORK_DIR / "watchdog_logs"          # log directory
HEARTBEAT_FILE = WORK_DIR / "pcd_heartbeat.txt" # heartbeat file (optional, see below)
NO_HEARTBEAT_TIMEOUT_SEC = 300                # heartbeat timeout threshold (no heartbeat for N seconds is considered deadlock, restart)
BACKOFF_BASE_SEC = 3                          # backoff base seconds for consecutive crashes
BACKOFF_MAX_SEC = 60                          # backoff max seconds
# =======================

def now(): return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def start_child():
    LOG_DIR.mkdir(exist_ok=True)
    logfile = LOG_DIR / (datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
    logf = open(logfile, "ab", buffering=0)

    cmd = [PYTHON_EXE, "-u", str(WORK_DIR / TARGET_SCRIPT)]
    env = os.environ.copy()
    print(f"[{now()}] spawn: {' '.join(cmd)}")

    # stdout/stderr
    p = subprocess.Popen(
        cmd,
        cwd=WORK_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        universal_newlines=True,
    )

    # open a thread to pipe output
    import threading
    def _pipe_output(proc, logfile_handle):
        for line in proc.stdout:
            sys.stdout.write(line)  # print to console
            logfile_handle.write(line.encode("utf-8", errors="ignore"))
            logfile_handle.flush()
    threading.Thread(target=_pipe_output, args=(p, logf), daemon=True).start()

    return p, logf, logfile


def kill_child_tree(p):
    if p and p.poll() is None:
        try:
            # Windows with <terminate> method
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        except Exception:
            pass

def heartbeat_is_stale():
    if not HEARTBEAT_FILE.exists():
        return False  # If you haven't connected the heartbeat, just rely on process exit judgment
    try:
        mtime = HEARTBEAT_FILE.stat().st_mtime
        return (time.time() - mtime) > NO_HEARTBEAT_TIMEOUT_SEC
    except Exception:
        return False

def main():
    consecutive_crashes = 0
    child = None; logf = None

    
    def _sig_handler(sig, frame):
        print(f"[{now()}] watchdog got signal {sig}, stopping child…")
        kill_child_tree(child)
        if logf: logf.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    while True:
        child, logf, logfile = start_child()

        # monitor child process & heartbeat
        while True:
            time.sleep(2)

            # 1) Is the process exited?
            rc = child.poll()
            if rc is not None:
                print(f"[{now()}] child exited rc={rc}, log={logfile}")
                consecutive_crashes += 1
                break

            # 2) Is the heartbeat stale (considered dead)?
            if heartbeat_is_stale():
                print(f"[{now()}] heartbeat stale (> {NO_HEARTBEAT_TIMEOUT_SEC}s). Restarting child.")
                kill_child_tree(child)
                consecutive_crashes += 1
                break

            # 3) Heartbeat normal?
            consecutive_crashes = 0

        # backoff and restart
        backoff = min(BACKOFF_BASE_SEC * (2 ** max(0, consecutive_crashes - 1)), BACKOFF_MAX_SEC)
        backoff = int(backoff)
        print(f"[{now()}] restarting in {backoff}s (consecutive crashes={consecutive_crashes}) …")
        try:
            if logf: logf.close()
        except Exception: pass
        time.sleep(backoff)

if __name__ == "__main__":
    main()
