# lidar_driver.py
import subprocess
import os
import signal
import time
import atexit

class LidarDriver:
    def __init__(self, launch_cmd):
        self.proc = None
        self.launch_cmd = launch_cmd
        self._start_lidar()
        atexit.register(self._emergency_stop)

    def _start_lidar(self):
        """Start the LiDAR process"""
        try:
            self.proc = subprocess.Popen(
                self.launch_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            time.sleep(3)  # Wait for LiDAR to initialize
            print(f"LiDAR started with PID: {self.proc.pid}")
        except Exception as e:
            print(f"Failed to start LiDAR: {e}")
            self.proc = None

    def stop(self):
        """Orderly stop of LiDAR"""
        self._kill_lidar()

    def _kill_lidar(self):
        """Kill the LiDAR process"""
        if self.proc and self.proc.poll() is None:
            try:
                print("Stopping LiDAR...")
                # Try SIGINT first (graceful shutdown)
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                time.sleep(2)
                
                # If still running, use SIGTERM
                if self.proc.poll() is None:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                    time.sleep(1)
                
                # Force kill if necessary
                if self.proc.poll() is None:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                
                self.proc.wait(timeout=5)
                print("LiDAR stopped successfully")
            except ProcessLookupError:
                pass  # Process already dead
            except Exception as e:
                print(f"Error stopping LiDAR: {e}")
            finally:
                self.proc = None

    def _emergency_stop(self):
        """Emergency stop called on program exit"""
        self._kill_lidar()