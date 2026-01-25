# test_servo.py
import serial
import time

# Test connection to Arduino
port = "/dev/ttyACM0"  # Adjust if needed
baud = 115200

print(f"Testing connection to Arduino on {port}...")

try:
    ser = serial.Serial(port, baud, timeout=2.0)
    time.sleep(2)  # Wait for Arduino reset
    
    # Clear buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # Wait for Arduino ready message
    print("Waiting for Arduino to send READY...")
    timeout = time.time() + 5
    while time.time() < timeout:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            print(f"Arduino: {line}")
            if "READY" in line:
                print("Arduino is ready!")
                break
    
    # Test sending angles
    test_angles = [-15, -10, -5, 0, 5, 10, 15]
    
    for angle in test_angles:
        command = f"ANGLE:{angle:.2f}\n"
        print(f"Sending: {command.strip()}")
        ser.write(command.encode())
        ser.flush()
        
        # Wait for response
        response = ser.readline().decode().strip()
        if response:
            print(f"Arduino response: {response}")
        else:
            print("No response from Arduino")
        
        time.sleep(1)  # Wait to see servo move
    
    # Center and close
    ser.write(b"ANGLE:0.00\n")
    time.sleep(1)
    ser.close()
    
    print("Test completed!")
    
except Exception as e:
    print(f"Error: {e}")