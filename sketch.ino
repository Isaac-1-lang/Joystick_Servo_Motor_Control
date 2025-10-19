#include <Servo.h>

#define x_pin A0
#define y_pin A1
#define hardcoded_vcc A2
#define hardcoded_gnd A3

Servo myServo;  // Create servo object

void setup() {
  Serial.begin(9600);

  // Power the joystick directly from Arduino
  pinMode(hardcoded_vcc, OUTPUT);
  pinMode(hardcoded_gnd, OUTPUT);
  digitalWrite(hardcoded_vcc, HIGH);  // Provide +5V
  digitalWrite(hardcoded_gnd, LOW);   // Provide GND

  myServo.attach(9);  // Connect servo signal pin to D9 (PWM)

  Serial.println("Joystick to Servo Control Started...");
}

void loop() {
  // Read analog values
  int x_val = analogRead(x_pin);  // X-axis (0 to 1023)
  int y_val = analogRead(y_pin);  // Y-axis (optional, if using 2 servos)

  // Map joystick X to servo angle (0°–180°)
  int servoAngle = map(x_val, 0, 1023, 0, 180);

  // Optional: Constrain angle just in case
  servoAngle = constrain(servoAngle, 0, 180);

  // Write angle to servo
  myServo.write(servoAngle);

  // Display data in Serial Monitor
  Serial.print("Joystick X: ");
  Serial.print(x_val);
  Serial.print("\tServo Angle: ");
  Serial.println(servoAngle);

  delay(15);  // Small delay for smooth motion
}
