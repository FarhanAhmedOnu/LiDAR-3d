/*
 * Arduino Code for Python ROS 2 Lidar 3D Scanner
 * * Instructions:
 * 1. Connect Servo Signal wire to Pin 9 (or change SERVO_PIN below).
 * 2. Connect Servo VCC to 5V (External power recommended for stability).
 * 3. Connect Servo GND to Arduino GND.
 * 4. Upload this sketch.
 */

#include <Servo.h>

Servo lidarServo;

// Configuration
const int SERVO_PIN = 9;
const int BAUD_RATE = 9600;      // Must match Python 'self.baud_rate'
const int MOVEMENT_DELAY = 250;  // Time (ms) to wait for servo to stabilize

void setup() {
  // Initialize Serial Connection
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(50); // Short timeout for faster parsing

  // Attach Servo
  lidarServo.attach(SERVO_PIN);
  
  // Set initial position to 90 (center)
  lidarServo.write(90);
  delay(500); // Allow initial movement
}

void loop() {
  // Check if Python has sent data
  if (Serial.available() > 0) {
    
    // Parse the integer sent by Python (e.g., "90\n" becomes 90)
    int targetAngle = Serial.parseInt();
    
    // Clear buffer of any remaining newline characters
    while (Serial.available() > 0) {
      Serial.read();
    }

    // Safety check: Keep angle within servo limits
    if (targetAngle >= 0 && targetAngle <= 180) {
      
      // 1. Move the Servo
      lidarServo.write(targetAngle);
      
      // 2. Wait for physical movement + stabilization
      // This is crucial. If the servo is still shaking when the Lidar scans,
      // the 3D cloud will look wavy/distorted.
      delay(MOVEMENT_DELAY);
      
      // 3. Send Confirmation back to Python
      // The Python script specifically waits for the string "Moved to"
      Serial.print("Moved to ");
      Serial.println(targetAngle);
    }
  }
}
