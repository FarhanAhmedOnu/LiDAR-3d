// sketch_firmata.ino
#include <Servo.h>
#include <Firmata.h>

// ============================
// CONFIGURATION
// ============================
const int SERVO_PIN = 9;

// Mechanical center of servo
const float SERVO_CENTER = 90.0;

// Sweep range (degrees from center)
const float SWEEP_MIN = -15.0;
const float SWEEP_MAX =  15.0;

// Step per update (degrees)
const float STEP_DEG = 0.5;

// Update rate (ms)
const unsigned long UPDATE_INTERVAL_MS = 30;

// ============================
// GLOBALS
// ============================
Servo tiltServo;

float tilt_deg = SWEEP_MIN;
float direction = 1.0;

unsigned long last_update = 0;

void setup() {
  Firmata.begin(57600);
  tiltServo.attach(SERVO_PIN);
  tiltServo.write(SERVO_CENTER);
  delay(500);
}

void loop() {
  // Process Firmata commands
  while (Firmata.available()) {
    Firmata.processInput();
  }
  
  unsigned long now = millis();

  if (now - last_update >= UPDATE_INTERVAL_MS) {
    last_update = now;

    // Update tilt angle
    tilt_deg += direction * STEP_DEG;

    if (tilt_deg >= SWEEP_MAX) {
      tilt_deg = SWEEP_MAX;
      direction = -1.0;
    }
    else if (tilt_deg <= SWEEP_MIN) {
      tilt_deg = SWEEP_MIN;
      direction = 1.0;
    }

    // Move servo
    float servo_angle = SERVO_CENTER + tilt_deg;
    tiltServo.write(servo_angle);

    // Send tilt angle via Firmata
    Firmata.sendAnalog(SERVO_PIN, tilt_deg * 1000); // Send as integer (scaled by 1000)
  }
}
