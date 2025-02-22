//  ***********************************************************************************
//  * Project: ESP32 Obstacle Avoidance Robot  
//  * Version: 1.0  
//  * Release Date: 01-22-2025  
//  * Author: Syntax-Surfer  
//  *  
//  * Project Summary:  
//  * - This program controls an autonomous robot using an ESP32, a servo motor,  
//  *   three ultrasonic sensors (front, left, and right), and an L298N motor driver.  
//  * - The robot moves forward while continuously scanning distances using the ultrasonic sensors.  
//  * - When an obstacle is detected in front, it reverses, scans left and right using a servo-mounted  
//  *   ultrasonic sensor, and determines the best direction to turn based on distance readings.  
//  * - If both sides are blocked, the robot continues reversing until a clear path is found.  
//  * - A motor speed ramp-up system ensures smooth acceleration and prevents sudden jerks.  
//  *  
//  * ---------------------------------------------------------------------------------
//  * Components Used:  
//  * - ESP32 Development Board  
//  * - L298N Motor Driver Module  
//  * - Ultrasonic Sensors (HC-SR04) ×3 (Front, Left, Right)  
//  * - Servo Motor (SG90 or MG995 for better stability)  
//  * - Two DC Motors (For wheels)  
//  * - Power Supply (Li-Ion Battery)  
//  * - Jumper Wires and Breadboard  
//  * ---------------------------------------------------------------------------------  
//  *  
//  * Development Environment:  
//  * - PlatformIO (Recommended for ESP32 development)  
//  * - VS Code with PlatformIO extension  
//  * ---------------------------------------------------------------------------------  
//  *  
//  * Libraries Required:  
//  * - ESP32Servo (For controlling the servo motor)  
//  * - NewPing (For ultrasonic sensor distance measurement)  
//  * ---------------------------------------------------------------------------------  
//  *  
//  * How to Run the Project (Using PlatformIO):  
//  * 1. Install **VS Code** and the **PlatformIO** extension.  
//  * 2. Create a new **PlatformIO project** and select **ESP32** as the board.  
//  * 3. Add the required libraries to `platformio.ini` or install them via PlatformIO.  
//  * 4. Connect all components according to the wiring diagram.  
//  * 5. Build and upload the code to the ESP32 using **PlatformIO Upload**.  
//  * 6. Power the ESP32 and motor driver with an external power source.  
//  * 7. Observe the robot's movement and object detection in real time.  
//  * ---------------------------------------------------------------------------------  
//  *  
//  * Adjustable Parameters:  
//  * - OBJECT_THRESHOLD_FRONT, OBJECT_THRESHOLD_LEFT, OBJECT_THRESHOLD_RIGHT (Distance in cm)  
//  * - OBJECT_DETECTION_DELAY (Time delay after object detection)  
//  * - LOOK_ANGLE (Maximum servo rotation angle)  
//  * - SPEED_INCREMENT (Motor speed ramp-up step)  
//  * ---------------------------------------------------------------------------------  
//  *  
//  * Key Features of Version 1.0:  
//  * ✅ Real-time object detection using three ultrasonic sensors.  
//  * ✅ Servo-based scanning to determine the best path.  
//  * ✅ Smooth motor acceleration with PWM-based speed control.  
//  * ✅ Intelligent obstacle avoidance with decision-making logic.  
//  * ✅ Code optimization for stability and performance.  
//  ***********************************************************************************

#include <ESP32Servo.h>
#include <NewPing.h>

// Pin assignments for ultrasonic sensors
// Pin Assignments
#define TRIG_PIN_FRONT 18
#define ECHO_PIN_FRONT 19
#define TRIG_PIN_LEFT 15
#define ECHO_PIN_LEFT 4
#define TRIG_PIN_RIGHT 23
#define ECHO_PIN_RIGHT 22

// Servo pin
#define SERVO_PIN 13

// Motor driver pins
#define LEFT_MOTOR_IN1 12
#define LEFT_MOTOR_IN2 14
#define RIGHT_MOTOR_IN3 27
#define RIGHT_MOTOR_IN4 26

// Maximum distance (in cm) for ultrasonic sensors
#define MAX_DISTANCE 200

// Distance threshold for triggering movement (adjustable)
int OBJECT_THRESHOLD_FRONT = 30;  // in cm
int OBJECT_THRESHOLD_SIDE = 18;   // in cm

// Delay after object detection (adjustable)
float OBJECT_DETECTION_DELAY = 0.05;  // Delay for stability (in seconds)

// Customizable angle for looking left and right
int LOOK_ANGLE = 70;  // Maximum angle for servo rotation

// Motor speed control variables
int motorSpeed = 0;    // Start with low speed (PWM) 
int maxSpeed = 200;    // Max PWM value for motor speed (0 to 255)
int speedIncrement = 5; // Speed increment per loop
int speedStart = 50;   // PWM value to start from (0 to 255)
int speedStepDelay = 20; // Delay for increasing speed (adjustable)

// Add a global flag to track motor speed ramp-up status
bool isMovingForward = false;

// Initialize ultrasonic sensors
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// Initialize servo
Servo myservo;

// Function prototypes
void stopMovement();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void scanLeftRight(int &combinedLeft, int &combinedRight);

void setup() {
  Serial.begin(115200);          // Start serial communication
  myservo.attach(SERVO_PIN);     // Attach servo to pin
  myservo.write(90);             // Set servo to center position

  // Set motor driver pins as outputs
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
}

void loop() {
  // Read distances from all three ultrasonic sensors
  int distanceFront = sonarFront.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();
  int distanceRight = sonarRight.ping_cm();

  // Handle cases where no echo is detected (distance will be 0)
  if (distanceFront == 0) distanceFront = MAX_DISTANCE;
  if (distanceLeft == 0) distanceLeft = MAX_DISTANCE;
  if (distanceRight == 0) distanceRight = MAX_DISTANCE;

  // Display sensor readings for debugging
  Serial.println("======================");
  Serial.printf("Front Distance: %d cm\n", distanceFront);
  Serial.printf("Left Distance: %d cm\n", distanceLeft);
  Serial.printf("Right Distance: %d cm\n", distanceRight);
  Serial.println("======================");

  // Add stabilization delay after detecting objects
  delay(OBJECT_DETECTION_DELAY * 1000);

  // Handle object detection logic
  if (distanceFront <= OBJECT_THRESHOLD_FRONT || 
      (distanceLeft <= OBJECT_THRESHOLD_SIDE && distanceRight <= OBJECT_THRESHOLD_SIDE)) {
    Serial.println("Object detected in front or on both sides. Reversing...");
    stopMovement();
    moveBackward();
    delay(1000);  // Reverse for 1 second
    stopMovement();

    // Perform left-right scan
    int combinedLeft, combinedRight;
    scanLeftRight(combinedLeft, combinedRight);

    // Display combined data for debugging
    Serial.printf("Combined Left Data: %d cm\n", combinedLeft);
    Serial.printf("Combined Right Data: %d cm\n", combinedRight);

    // Decide direction based on combined data
    if (combinedLeft > OBJECT_THRESHOLD_SIDE || combinedRight > OBJECT_THRESHOLD_SIDE) {
        if (combinedLeft > combinedRight) {
            Serial.println("Turning left...");
            turnLeft();
        } else {
            Serial.println("Turning right...");
            turnRight();
        }
    } else {
        Serial.println("Both sides blocked or too short. Reversing again...");
        moveBackward();
        delay(100);  // Reverse again for 1 second
    }
  } else if (distanceLeft <= OBJECT_THRESHOLD_SIDE) {
    // Object detected on the left side
    Serial.println("Object detected on the left. Turning right...");
    turnRight();
  } else if (distanceRight <= OBJECT_THRESHOLD_SIDE) {
    // Object detected on the right side
    Serial.println("Object detected on the right. Turning left...");
    turnLeft();
  } else {
    // No objects detected: Move forward
    Serial.println("No objects detected. Moving forward...");
    moveForward();
  }

  // Short delay to avoid flooding the serial output
  delay(50);
}

void scanLeftRight(int &combinedLeft, int &combinedRight) {
  int leftSensor, rightSensor, frontSensorLeft, frontSensorRight;

  // Look left
  myservo.write(90 + LOOK_ANGLE);
  delay(500);  // Wait for servo to reach position
  leftSensor = sonarLeft.ping_cm();
  frontSensorLeft = sonarFront.ping_cm();
  if (leftSensor == 0) leftSensor = MAX_DISTANCE;
  if (frontSensorLeft == 0) frontSensorLeft = MAX_DISTANCE;

  // Look right
  myservo.write(90 - LOOK_ANGLE);
  delay(500);  // Wait for servo to reach position
  rightSensor = sonarRight.ping_cm();
  frontSensorRight = sonarFront.ping_cm();
  if (rightSensor == 0) rightSensor = MAX_DISTANCE;
  if (frontSensorRight == 0) frontSensorRight = MAX_DISTANCE;

  // Return to center
  myservo.write(90);
  delay(500);  // Wait for servo to return to cen
  // Combine data from front and side sensors
  combinedLeft = min(leftSensor, frontSensorLeft);
  combinedRight = min(rightSensor, frontSensorRight);
}

void stopMovement() {
  analogWrite(LEFT_MOTOR_IN1, 0);
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN3, 0);
  analogWrite(RIGHT_MOTOR_IN4, 0);
  isMovingForward = false;  // Reset the flag
  Serial.println("Motors stopped");
}


void moveForward() {
  if (!isMovingForward) {
    // Ramp up speed from low to max PWM duty cycle
    for (motorSpeed = speedStart; motorSpeed <= maxSpeed; motorSpeed += speedIncrement) {
      analogWrite(LEFT_MOTOR_IN1, motorSpeed);
      analogWrite(LEFT_MOTOR_IN2, 0);   // Forward direction for left motor
      analogWrite(RIGHT_MOTOR_IN3, motorSpeed);
      analogWrite(RIGHT_MOTOR_IN4, 0);  // Forward direction for right motor
      delay(speedStepDelay);  // Delay for smooth acceleration
    }
    isMovingForward = true;  // Mark that the motor has reached max speed
    Serial.println("Moving forward at max speed");
  } else {
    // Keep running at max speed
    analogWrite(LEFT_MOTOR_IN1, maxSpeed);
    analogWrite(LEFT_MOTOR_IN2, 0);   // Forward direction for left motor
    analogWrite(RIGHT_MOTOR_IN3, maxSpeed);
    analogWrite(RIGHT_MOTOR_IN4, 0);  // Forward direction for right motor
    Serial.println("Continuing forward at max speed");
  }
}

void moveBackward() {
  // Ramp up speed from low to max PWM duty cycle
  for (motorSpeed = speedStart; motorSpeed <= maxSpeed; motorSpeed += speedIncrement) {
    analogWrite(LEFT_MOTOR_IN1, 0);   // Backward direction for left motor
    analogWrite(LEFT_MOTOR_IN2, motorSpeed);
    analogWrite(RIGHT_MOTOR_IN3, 0);  // Backward direction for right motor
    analogWrite(RIGHT_MOTOR_IN4, motorSpeed);
    // delay(speedStepDelay);  // Delay for smooth acceleration
  }
  Serial.println("Moving backward at max speed");
}

void turnLeft() {
  Serial.println("Turning left at full power...");
  
  // Apply full power to turn left
  analogWrite(LEFT_MOTOR_IN1, 0);   
  analogWrite(LEFT_MOTOR_IN2, maxSpeed);  // Reverse left motor
  analogWrite(RIGHT_MOTOR_IN3, maxSpeed); // Forward right motor
  analogWrite(RIGHT_MOTOR_IN4, 0);

  delay(600);  // Adjust delay based on turning speed
  stopMovement();
}

void turnRight() {
  Serial.println("Turning right at full power...");

  // Apply full power to turn right
  analogWrite(LEFT_MOTOR_IN1, maxSpeed);  // Forward left motor
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN3, 0);
  analogWrite(RIGHT_MOTOR_IN4, maxSpeed); // Reverse right motor

  delay(600);  // Adjust delay based on turning speed
  stopMovement();
}