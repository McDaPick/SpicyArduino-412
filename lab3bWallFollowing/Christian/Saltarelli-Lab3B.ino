// [Lab 3B] PID Wall Following
//   - This program is an introduction to utilizing the PID algorithm to smoothly guide our
//   robot as it follows along a wall. From collecting US readings from multiple angles this program
//  allows your robot to guide itself following the guidelines we set fourth.
//
// @author Christian Saltarelli
// @date 3-21-22
#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Servo servo;
Motors motors;

// Switches
const boolean HEAD_DEBUG = false;
const boolean TIMING_DEBUG = false;
const boolean US_DEBUG = false;
const boolean MOTOR_DEBUG = false;
const boolean TEST_DEBUG = true;

const boolean MOTOR_ON = true; 
const boolean US_ON = true;

// Desired Positions {Left, Front}
const int NUM_POSITIONS = 2;
const int NUM_CONSTANTS = 3;
const double desiredStates[NUM_POSITIONS] = {(double) 15, (double) 25};

// PID Constants {Left, Front} e.g [0] = {kp, ki, kd}       // 1, 0.001, 6      5, 23, 1.5
const double PID_CONSTANTS[NUM_POSITIONS][NUM_CONSTANTS] = {{1, 0.001, 8}, {1, 0.05, 22}};
double runningTotals[NUM_POSITIONS] = {0, 0};

// Servo Timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 445; // Interval to Update Head

// Servo Constants
const int SERVO_PIN = 20;                  // Left, Front
const int SERVO_POSITIONS[NUM_POSITIONS] = {180, 93};

// Servo Data
boolean servoDirectionClockwise = true;
int currentHeadPosition = 0; // Index
boolean prioritizeFront = false;

// Initialize Ultrasonic
const int ECHO_PIN = 12;
const int TRIG_PIN = 18;

// Ultrasonic Max Distance
const float MAX_DISTANCE = 80.0;
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;

// Ultrasonic Timing
unsigned long usCm;
const unsigned long US_WAIT_PERIOD = 200;  // Interval to Check US
boolean usReadFlag = false;                // Ensures 1 reading from US 

// Distance Readings {Left, Front}
float distances[NUM_POSITIONS] = {0, 0};

// Error Rates {Left, Front}
float errorRates[NUM_POSITIONS] = {0, 0};
float prevErrorRates[NUM_POSITIONS] = {0, 0};

// Motor Constraints
const float MOTOR_BASE_SPEED = 40.0;
const float MOTOR_MIN_SPEED = 20;
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

/** TESTING GLOBALS **/
float leftSpeed;
float rightSpeed;
double proportional;
double integral;
float derivative;

void setup() {
  // Update Pin Mapping + Motor + Servo Positioning
  Serial.begin(57600);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  servo.attach(SERVO_PIN);
  servo.write(SERVO_POSITIONS[0]); // Default 180

  delay(3000);
  buzzer.play("c32");
}

void loop() {  
  // Update US Position
  moveHead();
  
  // Get US Reading at respective Position
  usReadCm();
}

/**
 * moveHead()
 * - Used to direct our servo motor
 * based on direction and our positions
 * container. 
 */
void moveHead() {
  headCm = millis();
  if (headCm > headPm + HEAD_MOVEMENT_PERIOD) {
    // Set Next Head Position
    if (servoDirectionClockwise) {
      if (currentHeadPosition >= (NUM_POSITIONS - 1)) {
        servoDirectionClockwise = !servoDirectionClockwise;
        currentHeadPosition--;
        
      } else {
        currentHeadPosition++;
      }
    
    } else {
      if (currentHeadPosition <= 0) {
        servoDirectionClockwise= !servoDirectionClockwise;
        currentHeadPosition++;
      
      } else {
        currentHeadPosition--;
      }
    }

    servo.write(SERVO_POSITIONS[currentHeadPosition]);

    if (TIMING_DEBUG) {
      Serial.print("Move head initiated: ");
      Serial.println(headCm);
    }

    // Debug Flag Output
    if (HEAD_DEBUG) {
      Serial.print("Angle: ");
      Serial.print(currentHeadPosition);
      Serial.print(" - ");
      Serial.println(SERVO_POSITIONS[currentHeadPosition]);
    }


    // Update Previous Millis
    headPm = headCm;

    // Update US Read Flag
    usReadFlag = false;
  }
}

/**
 * usReadCm()
 * - Used to initiate our US and
 *   determine the respective distance (cm)
 *   from any returned sound waves.
 */
void usReadCm() {
  usCm = millis();
  if (usCm > headPm + US_WAIT_PERIOD && !usReadFlag) {
     if (TIMING_DEBUG) {
      Serial.print("US read initiated: ");
      Serial.println(usCm);
     }

     if (US_ON) {
      // Clear TRIG_PIN
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
  
      // Initiate our TRIG_PIN to HIGH (active) for 10s
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
  
      // Get Sound Travel Time via ECHO_PIN
      long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
      // Calculate Distance w/ ToF for Respective Angle
      distances[currentHeadPosition] = duration * 0.034 / 2;
  
      // Apply Respective Limits
      if (distances[currentHeadPosition] > MAX_DISTANCE) distances[currentHeadPosition] = MAX_DISTANCE;
      if (distances[currentHeadPosition] == 0) distances[currentHeadPosition] = desiredStates[currentHeadPosition];
  
      if (US_DEBUG) {
        Serial.print("Distance readings: [");
        for(int i = 0; i < NUM_POSITIONS; i++) {
          if (i == currentHeadPosition) {
            Serial.print("N ");
            Serial.print(distances[i]);
          } else {
            Serial.print(distances[i]);          
          }
          if (i < NUM_POSITIONS - 1) Serial.print(" - ");
        }
        Serial.println(" ]");
      }
  
      // Update US Read Flag
      usReadFlag = true;
  
      // Update Motors
      if (currentHeadPosition != 1) {
        if (!prioritizeFront) {
          updateMotors(updatePid()); 
        }
      } 
      
      if (currentHeadPosition == 1 && distances[currentHeadPosition] < desiredStates[currentHeadPosition]) {
        // Flip Priority Flag for Front US Position
        prioritizeFront = true; 
       
        // Only Update Motors on Front reading if our current distance is closer than our desired dist -- otherwise ignore
        updateMotors(updatePid());
        
      } else if (currentHeadPosition == 1 && distances[currentHeadPosition] == MAX_DISTANCE) {
        // Flip Priority Flag Back for Front US Position
        prioritizeFront = false;
      } else {
        Serial.println();
        Serial.print("Skipped Front Sensor Reading!");
        Serial.println();
      }
    }
  }
}

/**
 * updatePid()
 * - Used to calculate our error rate
 *   based on the components of the PID
 *   algorithm.
 */
float updatePid() {
  // Calculate Respective Error for Current Position
  errorRates[currentHeadPosition] = desiredStates[currentHeadPosition] - distances[currentHeadPosition];
  
  // Calculate Proportional [Kp * e]
  proportional = PID_CONSTANTS[currentHeadPosition][0] * errorRates[currentHeadPosition];

  // Calculate Integral Correction [ki * errorSum] + Update RunningTotal (ki)
  runningTotals[currentHeadPosition] += errorRates[currentHeadPosition];
  integral = PID_CONSTANTS[currentHeadPosition][1] * runningTotals[currentHeadPosition];

  // Calculate Derivative [Kd * (e - prev_e)]
  derivative = PID_CONSTANTS[currentHeadPosition][2] * (errorRates[currentHeadPosition] - prevErrorRates[currentHeadPosition]);

  // Update Previous Error Rate
  prevErrorRates[currentHeadPosition] = errorRates[currentHeadPosition];

  // Return the calculated Result
  return (proportional + integral + derivative);
}

/**
 * updateMotors(result)
 * - Takes the given result from our
 *   PID algorithm and applies it to 
 *   each of our respective motors.
 */
void updateMotors(float result) {
  // Start w/ BASE_SPEED
  leftSpeed = MOTOR_BASE_SPEED;
  rightSpeed = MOTOR_BASE_SPEED;

  // Calculate Magnitude
  float magnitude = (float)(result) / DISTANCE_FACTOR;

  leftSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
  rightSpeed = MOTOR_BASE_SPEED + (magnitude * MOTOR_FACTOR);

  // Lower Limit Check
  if (leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
  if (rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

  if (MOTOR_DEBUG) {
    Serial.print("Left: ");
    Serial.println(leftSpeed);
    Serial.print("Right: ");
    Serial.println(rightSpeed);
  }

  if (MOTOR_ON) {
    // Update Motor Speeds
    motors.setSpeeds(leftSpeed, rightSpeed);  
  }
}
