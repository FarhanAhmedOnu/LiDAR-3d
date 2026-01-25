# servo_controller.py
import serial
import math
import threading
import time

class ServoController:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.current_tilt_rad = 0.0
        self.running = True
        self.servo_ready = False
        self._shutdown_called = False
        
        # Wait for Arduino to initialize
        print("Waiting for Arduino...")
        time.sleep(2)  # Give Arduino time to boot
        
        # Clear any leftover data
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        # Check for ready signal
        start_time = time.time()
        while time.time() - start_time < 5.0:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode().strip()
                print(f"Arduino says: {line}")
                if "READY" in line:
                    self.servo_ready = True
                    print("Arduino is ready")
                    break
        
        if not self.servo_ready:
            print("Warning: Arduino not ready, but continuing...")
        
        # Start sweep control thread
        self.sweep_thread = threading.Thread(target=self._sweep_control, daemon=True)
        self.sweep_thread.start()
    
    def _sweep_control(self):
        """Control the servo sweep pattern"""
        sweep_min = -15.0
        sweep_max = 15.0
        step = 0.5
        current_angle = sweep_min
        direction = 1.0
        
        while self.running:
            try:
                # Update angle
                current_angle += direction * step
                
                # Check limits
                if current_angle >= sweep_max:
                    current_angle = sweep_max
                    direction = -1.0
                elif current_angle <= sweep_min:
                    current_angle = sweep_min
                    direction = 1.0
                
                # Send angle command to Arduino
                command = f"ANGLE:{current_angle:.2f}\n"
                self.ser.write(command.encode())
                self.ser.flush()  # Make sure it's sent immediately
                
                # Update current tilt
                self.current_tilt_rad = math.radians(current_angle)
                
                # Wait a bit for the servo to move
                time.sleep(0.03)  # ~30ms delay
                
            except Exception as e:
                if self.running:  # Only print error if we're still running
                    print(f"Error in sweep control: {e}")
                break
    
    def get_tilt_rad(self):
        return self.current_tilt_rad
    
    def _emergency_cleanup(self):
        """Center servo and cleanup on exit"""
        if self._shutdown_called:
            return
        
        print("Emergency cleanup - centering servo...")
        try:
            # Send center command
            self.ser.write(b"ANGLE:0.00\n")
            time.sleep(0.5)
        except:
            pass
        
        try:
            self.ser.close()
        except:
            pass
    
    def close(self):
        """Graceful shutdown"""
        if self._shutdown_called:
            return
        
        self._shutdown_called = True
        print("Closing servo controller...")
        self.running = False
        
        # Wait for sweep thread to finish
        if self.sweep_thread.is_alive():
            self.sweep_thread.join(timeout=2.0)
        
        # Do cleanup
        self._emergency_cleanup()