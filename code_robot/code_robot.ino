#include <PS4Controller.h>
#include <ESP32Servo.h>
#include "driver/ledc.h"

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

// LEDC Channel Definitions
#define LEDC_CHANNEL_RIGHT LEDC_CHANNEL_0
#define LEDC_CHANNEL_LEFT LEDC_CHANNEL_1

// LEDC Timer Definitions
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY 5000 // 5 kHz frequency
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution

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
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
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
    delay(10);
    handleDCMotorProtocol();
    delay(10);
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
  // Configure LEDC timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = LEDC_RESOLUTION,
    .timer_num = LEDC_TIMER,
    .freq_hz = LEDC_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Configure LEDC channel for right motor
  ledc_channel_config_t ledc_channel_right = {
    .gpio_num = PWM_RIGHT_PIN,
    .speed_mode = LEDC_MODE,
    .channel = LEDC_CHANNEL_RIGHT,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel_right);

  // Configure LEDC channel for left motor
  ledc_channel_config_t ledc_channel_left = {
    .gpio_num = PWM_LEFT_PIN,
    .speed_mode = LEDC_MODE,
    .channel = LEDC_CHANNEL_LEFT,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel_left);

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
  const float buttonStep1 = 0.05;   // Adjust step size for buttons

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
}

void handleDCMotorProtocol() {
  const int pwm_min = 90;
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

  if (pwmRight >= 0) {
    digitalWrite(RIGHT_OUTPUT_3_PIN, HIGH);
    digitalWrite(RIGHT_OUTPUT_4_PIN, LOW);
  } else {
    digitalWrite(RIGHT_OUTPUT_3_PIN, LOW);
    digitalWrite(RIGHT_OUTPUT_4_PIN, HIGH);
  }

  if (pwmRight >= pwm_min) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, abs(pwmRight));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
  } else if (pwmRight >= DEAD_ZONE) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, pwm_min);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
  } else {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, abs(pwmRight));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
  }

  if (pwmLeft >= 0) {
    digitalWrite(LEFT_OUTPUT_1_PIN, HIGH);
    digitalWrite(LEFT_OUTPUT_2_PIN, LOW);
  } else {
    digitalWrite(LEFT_OUTPUT_1_PIN, LOW);
    digitalWrite(LEFT_OUTPUT_2_PIN, HIGH);
  }

  if (pwmLeft >= pwm_min) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, abs(pwmLeft));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);
  } else if (pwmLeft >= DEAD_ZONE) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, pwm_min);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);
  } else {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, abs(pwmLeft));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);
  }
}