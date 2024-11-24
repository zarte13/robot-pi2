#include <PS4Controller.h>
#include <ESP32Servo.h>

// Pin Definitions
#define CONTINUOUS_SERVO_PIN 33
#define SERVO_180_PIN_1 25
#define SERVO_180_PIN_2 14

#define PWM_RIGHT_PIN 4
#define RIGHT_OUTPUT_4_PIN 16
#define RIGHT_OUTPUT_3_PIN 17
#define LEFT_OUTPUT_2_PIN 5
#define LEFT_OUTPUT_1_PIN 18
#define PWM_LEFT_PIN 19

// Servo Objects
Servo continuousServo;
Servo servo180_1;
Servo servo180_2;

// Protocol State
enum Protocol { SERVO, DC_MOTOR };
Protocol currentProtocol = SERVO;

// Joystick Dead Zone
const int DEAD_ZONE = 20;

// Servo Parameters
int continuousServoPulseWidth = 1500; // Neutral pulse width
int servo180_1PulseWidth = 1500; // Neutral position
int servo180_2PulseWidth = 1500; // Neutral position

// PWM Parameters
int pwmRight = 0;
int pwmLeft = 0;

// Keep-alive interval (in milliseconds)
const unsigned long KEEP_ALIVE_INTERVAL = 5000;
unsigned long lastKeepAliveTime = 0;


// Function Prototypes
void setupServos();
void setupDCMotors();
void handleServoProtocol();
void handleDCMotorProtocol();
void switchProtocol();

void setup() {
  Serial.begin(115200);
  PS4.begin(); // Initialize PS4 controller
  setupServos();
  setupDCMotors();
}

void loop() {
  if (PS4.isConnected()) {
    Serial.println("PS4 Controller is connected.");

    // Send a keep-alive signal every KEEP_ALIVE_INTERVAL milliseconds
    unsigned long currentTime = millis();
    if (currentTime - lastKeepAliveTime >= KEEP_ALIVE_INTERVAL) {
      PS4.setLed(0, 0, 255); // Example keep-alive command: set LED color
      lastKeepAliveTime = currentTime;
    }
    handleServoProtocol();
  } else {
    Serial.println("Waiting for PS4 Controller...");
    delay(1000); // Delay to prevent spamming the serial monitor
  }
}

void setupServos() {
  continuousServo.attach(CONTINUOUS_SERVO_PIN);
  servo180_1.attach(SERVO_180_PIN_1);
  servo180_2.attach(SERVO_180_PIN_2);
}

void setupDCMotors() {
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(RIGHT_OUTPUT_4_PIN, OUTPUT);
  pinMode(RIGHT_OUTPUT_3_PIN, OUTPUT);
  pinMode(LEFT_OUTPUT_2_PIN, OUTPUT);
  pinMode(LEFT_OUTPUT_1_PIN, OUTPUT);
}

void handleServoProtocol() {
  // Parameters for 180-degree servos
  const int pulseWidth180 = 2500; // Adjusted max pulse width
  const int pulseWidth0 = 500;   // Min pulse width
  const float joystickStep = 0.035; // Adjust step size for joystick
  const float buttonStep = 0.05;
  const float buttonStep1 = 20;   // Adjust step size for buttons

  const int pwm_min = 90;

  // Continuous Rotation Servo Control (Left Joystick)
  int rightJoystickY = PS4.RStickY();
  float leftAxisValue = rightJoystickY / 128.0; // Normalize joystick value to -1.0 to 1.0

  if (abs(leftAxisValue) > 0.13) {
    continuousServoPulseWidth = 1500 + (500 * leftAxisValue); // Adjust pulse width based on joystick
  } else {
    continuousServoPulseWidth = 1500; // Stop
  }
  continuousServo.writeMicroseconds(continuousServoPulseWidth);

  Serial.print("Left Joystick Y: ");
  Serial.print(rightJoystickY);
  Serial.print(", Continuous Servo Pulse Width: ");
  Serial.println(continuousServoPulseWidth);



  // 180-Degree Servo Control (L2 and R2 Buttons)
  float l2Value = PS4.L2(); // Read L2 value as a float between 0 and 128
  float r2Value = PS4.R2(); // Read R2 value as a float between 0 and 128

  Serial.print("L2 Value: ");
  Serial.print(l2Value);
  Serial.print(", R2 Value: ");
  Serial.println(r2Value);

 

  if (l2Value > 0.1 && r2Value > 0.1) {
    // Both buttons pressed, do not rotate
    // Hold position
  } else if (l2Value > 0.1) {
    // Move toward 0 degrees
    servo180_1PulseWidth += -servo180_1PulseWidth * l2Value * buttonStep;
  } else if (r2Value > 0.1) {
    // Move toward 180 degrees
    servo180_1PulseWidth += servo180_1PulseWidth * r2Value * buttonStep;
  }

  servo180_1PulseWidth = constrain(servo180_1PulseWidth, pulseWidth0, pulseWidth180);

  servo180_1.writeMicroseconds(servo180_1PulseWidth);

  Serial.print("Servo 180_1 Pulse Width: ");
  Serial.println(servo180_1PulseWidth);

  // 180-Degree Servo Control (L2 and R2 Buttons)
  float l1Value = PS4.L1(); // Read L2 value as a float between 0 and 1
  float r1Value = PS4.R1(); // Read R2 value as a float between 0 and 1

  Serial.print("L1 Value: ");
  Serial.print(l1Value);
  Serial.print(", R1 Value: ");
  Serial.println(r1Value);

  if (l1Value > 0.1 && r1Value > 0.1) {
    // Both buttons pressed, do not rotate
    // Hold position

  } else if (l1Value > 0.1) {
    // Move toward 0 degrees
    servo180_2PulseWidth += -servo180_2PulseWidth * l1Value * buttonStep1;

  } else if (r1Value > 0.1) {
    // Move toward 180 degrees
    servo180_2PulseWidth += servo180_2PulseWidth * r1Value * buttonStep1;
   
  }
  servo180_2PulseWidth = constrain(servo180_2PulseWidth, pulseWidth0, pulseWidth180);

  servo180_2.writeMicroseconds(servo180_2PulseWidth);

  Serial.print("Servo 180_2 Pulse Width: ");
  Serial.println(servo180_2PulseWidth);

    // Read Joystick Inputs
  int forwardBackward = PS4.LStickY();
  int turning = PS4.LStickX() * 0.7;


  // Apply Dead Zone
  if (abs(forwardBackward) < DEAD_ZONE) forwardBackward = 0;
  if (abs(turning) < DEAD_ZONE) turning = 0;

  // Calculate Motor Speeds
  pwmRight = map(forwardBackward - turning, -128, 128, -255, 255);
  pwmLeft = map(forwardBackward + turning, -128, 128, -255, 255);

  // Clamp Speeds
  pwmRight = constrain(pwmRight, -255, 255);
  pwmLeft = constrain(pwmLeft, -255, 255);

  // Set Motor Directions and Speeds
  if (pwmRight >= 0) {
    digitalWrite(RIGHT_OUTPUT_3_PIN, HIGH);
    digitalWrite(RIGHT_OUTPUT_4_PIN, LOW);
  } else {
    digitalWrite(RIGHT_OUTPUT_3_PIN, LOW);
    digitalWrite(RIGHT_OUTPUT_4_PIN, HIGH);
  }
  if (pwmLeft >= pwm_min) {
    analogWrite(PWM_RIGHT_PIN, abs(pwmRight));
  } else if (pwmLeft >= DEAD_ZONE) {
    analogWrite(PWM_RIGHT_PIN, pwm_min);
  } else {
    analogWrite(PWM_RIGHT_PIN, abs(pwmRight));
  }
  

  if (pwmLeft >= 0) {
    digitalWrite(LEFT_OUTPUT_1_PIN, HIGH);
    digitalWrite(LEFT_OUTPUT_2_PIN, LOW);
  } else {
    digitalWrite(LEFT_OUTPUT_1_PIN, LOW);
    digitalWrite(LEFT_OUTPUT_2_PIN, HIGH);
  }

  if (pwmLeft >= pwm_min) {
    analogWrite(PWM_LEFT_PIN, abs(pwmLeft));
  } else if (pwmLeft >= DEAD_ZONE) {
    analogWrite(PWM_LEFT_PIN, pwm_min);
  } else {
    analogWrite(PWM_LEFT_PIN, abs(pwmLeft));
  }

  Serial.print("Forward/Backward: ");
  Serial.print(forwardBackward);
  Serial.print(", Turning: ");
  Serial.print(turning);
  Serial.print(", PWM Right: ");
  Serial.print(pwmRight);
  Serial.print(", PWM Left: ");
  Serial.println(pwmLeft);
}
