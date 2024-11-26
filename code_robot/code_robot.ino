#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <cmath>

// Pin Definitions
#define CONTINUOUS_SERVO_PIN 33
#define SERVO_180_PIN_1 25
#define SERVO_180_PIN_2 12

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



// Joystick Dead Zone
const int DEAD_ZONE = 20;

// Servo Parameters
int continuousServoPulseWidth = 1500; // Neutral pulse width
int servo180_1PulseWidth = 90; // Neutral position
int servo180_2PulseWidth = 90; // Neutral position

// PWM Parameters
int pwmRight = 0;
int pwmLeft = 0;
float leftAxisValuePrev = 2;

// Keep-alive interval (in milliseconds)
const unsigned long KEEP_ALIVE_INTERVAL = 5000;
unsigned long lastKeepAliveTime = 0;


// Function Prototypes
void setupServos();
void setupDCMotors();
void handleServoProtocol();
void switchProtocol();
void rotateContinuousServo(Servo servoName, float stickValue);
void rotate180Servo(Servo servoName, float stickValue);


void setup() {
  Serial.begin(57600);
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
    delay(100);
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



void rotateContinuousServo(Servo servoName, float stickValue){

  if (abs(stickValue) > 0.13) {
    continuousServoPulseWidth = 1500 + (500 * stickValue); // Adjust pulse width based on joystick
  } else {
    continuousServoPulseWidth = 1500; // Stop
  }
  servoName.writeMicroseconds(continuousServoPulseWidth);
}

// void rotate180Servo(Servo servoName, float rotation){
//   int angle = servoName.read();
//   int continuousServoAngle = angle + rotation;
//   continuousServoAngle = constrain(continuousServoAngle, 0, 180);
//   servoName.write(continuousServoAngle);

// }

void rotate180Servo(Servo servoName, float rotation) {
  static float velocity = 0;
  int currentAngle = servoName.read();
  
  // Gradual acceleration
  float acceleration = rotation * 0.5; // Adjust as needed
  velocity += acceleration;
  
  // Add damping to prevent overshooting
  velocity *= 0.9; 
  
  int newAngle = currentAngle + velocity;
  newAngle = constrain(newAngle, 0, 180);
  
  servoName.write(newAngle);
}

void handleServoProtocol() {
  // Parameters for 180-degree servos
  const int pulseWidth180 = 2500; // Adjusted max pulse width
  const int pulseWidth0 = 500;   // Min pulse width
  const float joystickStep = 0.035; // Adjust step size for joystick
  const float buttonStep = 2;
  const float buttonStep1 = 2;   // Adjust step size for buttons

  const int pwm_min = 90;
  int servo1Rotation = 0;
  int servo2Rotation = 0;

  // Continuous Rotation Servo Control (Left Joystick)
  int rightJoystickY = PS4.RStickY();
  float leftAxisValue = rightJoystickY / 128.0; // Normalize joystick value to -1.0 to 1.0


  if (fabs(leftAxisValue - leftAxisValuePrev) <= 0.03) {
    rotateContinuousServo(continuousServo, leftAxisValue);
  }
  leftAxisValuePrev = leftAxisValue;
  Serial.print("Left Joystick Y: ");
  Serial.print(rightJoystickY);
  Serial.print(", Continuous Servo Pulse Width: ");
  Serial.println(continuousServoPulseWidth);



  // 180-Degree Servo Control (L2 and R2 Buttons)
  float l2Value = PS4.L2(); // Read L2 value as a float between 0 and 1
  float r2Value = PS4.R2(); // Read R2 value as a float between 0 and 1

  Serial.print("L2 Value: ");
  Serial.print(l2Value);
  Serial.print(", R2 Value: ");
  Serial.println(r2Value);

 


  if (l2Value > 0.1 && r2Value > 0.1) {
    // Both buttons pressed, do not rotate
    // Hold position
  } else if (l2Value > 0.1) {
    // Move toward 0 degrees
    servo1Rotation =  -l2Value * buttonStep;
  } else if (r2Value > 0.1) {
    // Move toward 180 degrees
    servo1Rotation = r2Value * buttonStep;
  }

  if (servo1Rotation != 0){
    rotate180Servo(servo180_1, servo1Rotation);
  }


  // 180-Degree Servo Control (L2 and R2 Buttons)
  int l1Value = PS4.L1(); // Read L2 value as a float between 0 and 1
  int r1Value = PS4.R1(); // Read R2 value as a float between 0 and 1

  Serial.print("L1 Value: ");
  Serial.print(l1Value);
  Serial.print(", R1 Value: ");
  Serial.println(r1Value);

  if (l1Value == 1 && r1Value == 1) {
    // Both buttons pressed, do not rotate
    // Hold position

  } else if (l1Value == 1) {
    // Move toward 0 degrees
    servo2Rotation =  -l1Value * buttonStep1;
  } else if (r1Value == 1) {
    // Move toward 180 degrees
    servo2Rotation =  r1Value * buttonStep1;
  }

  if (servo2Rotation != 0){
    rotate180Servo(servo180_2, servo2Rotation);
  }

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
