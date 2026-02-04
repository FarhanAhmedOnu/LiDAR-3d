#include <Servo.h>

const int SERVO_PIN = 9;
Servo tiltServo;

void setup() {
  Serial.begin(115200);
  tiltServo.attach(SERVO_PIN);
  
  // Move to 90 immediately on boot
  tiltServo.write(90); 
  delay(1000);
  Serial.println("READY"); 
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("ANGLE:")) {
      float target_angle = command.substring(6).toFloat();
      
      // Constrain to physical limits of your setup (e.g., 45 to 135)
      target_angle = constrain(target_angle, 45.0, 135.0);
      
      tiltServo.write(target_angle);
      
      Serial.print("ACK_ANGLE:");
      Serial.println(target_angle);
    }
  }
}
