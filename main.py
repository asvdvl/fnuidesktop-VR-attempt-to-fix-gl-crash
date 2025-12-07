#!/usr/bin/env python3
"""
VR Desktop - Wayland desktop mirror for SteamVR
Runs capture and VR display as separate processes
"""

import sys
import os
import signal
import subprocess
import time

script_dir = os.path.dirname(os.path.abspath(__file__))


def main():
    print("VR Desktop")
    print("=" * 50)
    print()

    capture_proc = None
    vr_proc = None

    def cleanup(signum=None, frame=None):
        print("\nShutting down...")
        for proc in [vr_proc, capture_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
        # Wait then force kill
        time.sleep(0.5)
        for proc in [vr_proc, capture_proc]:
            if proc and proc.poll() is None:
                proc.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # Start capture process
    capture_script = os.path.join(script_dir, 'test_wayland_capture.py')
    capture_proc = subprocess.Popen([sys.executable, capture_script])

    # Wait for shared memory file to appear
    shm_file = '/dev/shm/vr_desktop_frame'
    print("Waiting for capture to start...")
    for _ in range(100):  # 10 second timeout
        if os.path.exists(shm_file):
            break
        time.sleep(0.1)
    else:
        print("Capture failed to start")
        cleanup()
        return 1

    # Start VR display process
    vr_script = os.path.join(script_dir, 'vr_display.py')
    vr_proc = subprocess.Popen([sys.executable, vr_script])

    # Wait for either process to exit
    try:
        while True:
            if capture_proc.poll() is not None:
                print("Capture process exited")
                break
            if vr_proc.poll() is not None:
                print("VR display exited")
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    cleanup()
    return 0


if __name__ == '__main__':
    sys.exit(main())
