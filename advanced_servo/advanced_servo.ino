#include <Servo.h>

// ============================
// CONFIGURATION
// ============================
const int SERVO_PIN = 9;
const float SERVO_CENTER = 90.0;
const float SWEEP_MIN = -15.0;
const float SWEEP_MAX = 15.0;

// ============================
// GLOBALS
// ============================
Servo tiltServo;
bool servo_initialized = false;

void setup() {
  Serial.begin(115200);
  // Initialize immediately
  tiltServo.attach(SERVO_PIN);
  tiltServo.write(SERVO_CENTER);
  servo_initialized = true;
  delay(1000);  // Wait for servo to settle
  Serial.println("READY");  // Send ready signal
}

void loop() {
  // Listen for angle commands from Python
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Debug: echo what we received
    Serial.print("Received: ");
    Serial.println(command);
    
    // Parse angle command (format: "ANGLE:XX.XX")
    if (command.startsWith("ANGLE:")) {
      if (servo_initialized) {
        float angle = command.substring(6).toFloat();
        
        // Constrain to safe range
        angle = constrain(angle, SWEEP_MIN, SWEEP_MAX);
        
        // Convert to servo angle and move
        float servo_angle = SERVO_CENTER + angle;
        tiltServo.write(servo_angle);
        
        // Send confirmation back
        Serial.print("MOVED:");
        Serial.println(angle, 2);
      }
    }
  }
  
  // Small delay to prevent serial buffer overflow
  delay(10);
}
