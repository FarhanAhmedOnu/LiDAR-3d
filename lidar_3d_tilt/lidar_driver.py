# lidar_driver.py
import subprocess
import os
import signal
import time

class LidarDriver:
    def __init__(self, launch_cmd):
        self.proc = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        time.sleep(3)

    def stop(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except:
            pass
