#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Servo headServo;
Motors motors;

// Jarett Sutula, Michael Nielsen, Christian Salterelli
// Due 4/4/2022
// Lab 3b Objectives:
// - Implement PID to follow a wall at a set distance.
// - Robot should turn left/right around corners.

// Servo Constants and Timing
const int HEAD_SERVO_PIN = 20;
unsigned long headServoCm;
unsigned long headServoPm;
const unsigned long HEAD_SERVO_MOVEMENT_PERIOD = 320;
const int NUM_HEAD_POSITIONS = 3;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {90, 10, 10};
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

// Servo rotation is ~5 degrees off. so for a true 'straight 90 degree' angle, we need ~85.
// saving the buffer here lets us change it if we use a different servo or anything.
const int HEAD_ROTATION_BUFFER = -5;

// Need to give the servo time to move before we measure the distance!
// Needs to be longer than ~80 from previous lab. Servo is moving ~80 degrees instead of ~20.
unsigned long US_WAITING_TIME_LONG = 180;
unsigned long US_WAITING_TIME = US_WAITING_TIME_LONG;

// Debugs for printing and flags
const boolean DISTANCE_DEBUG = false;
const boolean HEAD_DEBUG = false;
const boolean SW_PID_DEBUG = true;
const boolean FW_PID_DEBUG = true;
const boolean MOTOR_DEBUG = false;
const boolean MOVE_MOTORS = true;
// flags to check if the servo is facing forwards and if the PIDs should be running
boolean headFacingFront = true;
boolean pidFlag = false;

// Ultrasonic Constants and Timing
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;
// This is the max distance (double the desired states for both PIDs)
const float MAX_SW_DISTANCE = 20.0;
const float MAX_FW_DISTANCE = 20.0;
// save a generic distance reading for every US reading, then apply it to the 
// appropriate PID after US readings finish
float distance = 0;
boolean usReadFlag = true;
unsigned long usCm;

// Side Wall PID Constants and Variables (sw)
float swDistance = 0;
const double swDesiredState = 10;
// proportional becomes unstable around ~4 with ki, kd = 0.
// oscillations dampen at ~.02 derivative. bad readings throw this off hard - keep low.
// movement is smooth against walls and around corners at around ~3.5, 0.05, 0.02 respectively.
const double swKp = 3.4;
const double swKi = 0.05;
const double swKd = .01;
double swKiTotalError = 0.0;
double swPreviousError = 0.0;
// to keep integral somewhat consistent, keep totalError between -30.0 and 30.0.
const double swIntegralLimit = 30;
// for side wall - one bad reading effectively ruins the smoothness and makes corners too sharp.
// this flag ignores (1) bad reading and continues previous PID result. If 2nd bad reading comes in,
// it will not be ignored. Flag only resets when side wall PID detects a wall, as to not string together
// multiple bad readings all being ignored.
boolean ignoreFirstBadReading = true;

// Front Wall PID Constants and Variables (fw)
float fwDistance = 0;
const double fwDesiredState = 10;
// proportional is a bad judge of movement here. Since we have 3:1 side:front readings, a difference between
// error of -5.00 and -4.00 means a drastic change in front-wall behavior. It needs to be consistent so keep low.
// integral will build up when front wall is not being detected - this should be high to make the first front-wall
// reading react strongly without being highly varied like kp and kd.
// derivative stays small as there is few oscillations in the (relatively) quick front-facing turn.
const double fwKp = .4;
const double fwKi = 5.75;
const double fwKd = .01;
double fwKiTotalError = 0.0;
double fwPreviousError = 0.0;
// Integral limit is MASSIVE here - setting it to 40 means that the total error hits around ~230 (due to ki).
// this will build up error as the robot doesn't detect a front wall, and when it does, this total error will guarantee
// a CONSISTENT ~90 degree turn against the front facing wall so the side-wall PID can pick up on the new wall.
const double fwIntegralLimit = 40;

// Motor Control
// a minimum motor speed for side-wall PID greatly improves corner reliability.
const double MOTOR_BASE_SPEED = 60;
const double MOTOR_MINIMUM_SPEED = 40;
float leftSpeed = MOTOR_BASE_SPEED;
float rightSpeed = MOTOR_BASE_SPEED;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(HEAD_POSITIONS[0] + HEAD_ROTATION_BUFFER);
  // reverse those motors since our guy is backwards!
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  // small delay to prep
  delay(3000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  moveHead();
  usReadCm();
  pidMotors();
}

void moveHead() {
  headServoCm = millis();
  if(headServoCm > headServoPm + HEAD_SERVO_MOVEMENT_PERIOD) {
    if(HEAD_DEBUG) {
      Serial.print("degrees: ");
      Serial.print(HEAD_POSITIONS[currentHeadPosition]);
    }

    // position head to the current position index.
    headServo.write(HEAD_POSITIONS[currentHeadPosition] + HEAD_ROTATION_BUFFER);

    // need to figure out what PID to use (front facing or wall facing)
    // Front facing is index 0, wall facing is indices 1, 2.
    if(currentHeadPosition == 0){
      headFacingFront = true;
    } else {
      headFacingFront = false;
    }

    // update the direction and index for next servo movement
    if(headDirectionClockwise) {
      if(currentHeadPosition >= NUM_HEAD_POSITIONS -1) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      }
      else {
        currentHeadPosition++;
      }
    }
    else {
      if(currentHeadPosition <= 0) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      }
      else {
        currentHeadPosition--;
      }
    }

    // reset millis for next period and reset flag
    headServoPm = headServoCm;
    usReadFlag = false;
  }
}

void usReadCm() {
  usCm = millis();
  // we ONLY want to take a reading if we've waited the US_WAITING_TIME
  // buffer to ensure it isn't looking for readings before the servo
  // is done moving to its new location.
  if(usCm > headServoPm + US_WAITING_TIME && !usReadFlag) {
    // clear the trig pin (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
  
    // Sets the TRIG_PIN HIGH (active) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  
    // Read ECHO_PIN, return the sound wave travel time in microseconds
    // note the duration, as it allows for reading up to max distance
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
  
    // divide by 2 from the total distance the sound has traveled
    // we only care about the one-way trip to the object!
    distance = duration * 0.034 / 2;

    // set distance reading for appropriate PID function
    if (headFacingFront) {
      // we have a reading from the front, use front-facing constants and variables.
      fwDistance = distance;
      if (fwDistance > MAX_FW_DISTANCE) fwDistance = MAX_FW_DISTANCE;
      if (fwDistance == 0) fwDistance = MAX_FW_DISTANCE;
    
    } else {
      // we have a reading from the side, use side-facing constants and variables.
      swDistance = distance;
      // apply limits
      if (swDistance > MAX_SW_DISTANCE) swDistance = MAX_SW_DISTANCE;
      if (swDistance == 0) swDistance = MAX_SW_DISTANCE;
    }
  
    // serial monitor output for distance (works for both PIDS)
    if(DISTANCE_DEBUG) {
      Serial.print("distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }

    // reset flag so it knows not to run this code until the flag
    // is set back to false when servo is changing angle in moveHead().
    usReadFlag = true;
    // a similar flag to make sure we only calculate PID after our reading code has been run.
    pidFlag = true;
  }
}

void pidMotors() {
  if(pidFlag) {
    // Let's figure out what direction we are facing, as it will impact what PID we are using.
    if(headFacingFront) {
      // Front-facing PID 
      // get the error based on where it should be - where it is.
      double error = fwDesiredState - fwDistance;
  
      // Calculate Proportional part of PID
      double proportional = fwKp * error;
  
      // add error to Integral part of PID
      fwKiTotalError += error;
      
      // Apply limits to Ki if needed.
      if (fwKiTotalError > fwIntegralLimit) {
        fwKiTotalError = fwIntegralLimit;
      } else if (fwKiTotalError < fwIntegralLimit * -1) {
        fwKiTotalError = fwIntegralLimit * -1;
      }
      
      // Calculate Integral part of PID
      double integral = fwKi * fwKiTotalError;
  
      // Calculate Derivative part of PID
      float derivative = fwKd * (error - fwPreviousError);
      // set previous error for next PID run
      fwPreviousError = error;
  
      // PID result from calculations
      float fwPidResult = proportional + integral + derivative;
  
      if(FW_PID_DEBUG){
        Serial.print("FW ----- error: ");
        Serial.print(error);
        Serial.print(" p: ");
        Serial.print(proportional);
        Serial.print(" i: ");
        Serial.print(integral);
        Serial.print(" d: ");
        Serial.print(derivative);
        Serial.print(" pid result: ");
        Serial.print(fwPidResult);
        Serial.print(" tE: ");
        Serial.println(fwKiTotalError);
      }

      // only change motor speed if we have detected a wall!
      // if not, then we ignore this - but still totaling up that error to ensure
      // the turn is consistent - see constants comments at top of file.
      // only do this if our reading is below the 'desired' state
      if(fwDistance < fwDesiredState) {
        leftSpeed = MOTOR_BASE_SPEED - fwPidResult;
        rightSpeed = MOTOR_BASE_SPEED + fwPidResult;
      }
      
    } else {
      if(!headFacingFront) {
        // if PID is being called and we aren't facing the front, activate wall-facing PID
        // get the error based on (where it should be) - (where it is)
        double error = swDesiredState - swDistance;

        // If our error is max (i.e no wall in sight or BAD READING)
        // check to see if we should be ignoring this reading (only once per stretch of bad readings)
        if(error == swDesiredState - MAX_SW_DISTANCE and ignoreFirstBadReading) {
            buzzer.play("c32");
            ignoreFirstBadReading = false;
            Serial.println("BEEP");
        } else {
          // if we have a VALID reading, reset the ignoring first bad reading! it helps our
          // oscillation smooth out by not having massive derivative jumps from 1 bad reading.
          if (error != swDesiredState - MAX_SW_DISTANCE) ignoreFirstBadReading = true;
          // Calculate Proportional part of PID
          double proportional = swKp * error;
      
          // add error to Integral part of PID
          swKiTotalError += error;
      
          // Apply limits to Ki if needed.
          if (swKiTotalError > swIntegralLimit) {
            swKiTotalError = swIntegralLimit;
          } else if (swKiTotalError < swIntegralLimit * -1) {
            // also apply to the negative side as well
            swKiTotalError = swIntegralLimit * -1;
          }
    
          // Calculate Integral part of PID
          double integral = swKi * swKiTotalError;
      
          // Calculate Derivative part of PID
          float derivative = swKd * (error - swPreviousError);
          // set previous error for next PID run
          swPreviousError = error;
      
          // PID result from calculations
          float swPidResult = proportional + integral + derivative;
      
          if(SW_PID_DEBUG){
            Serial.print("SW ----- error: ");
            Serial.print(error);
            Serial.print(" p: ");
            Serial.print(proportional);
            Serial.print(" i: ");
            Serial.print(integral);
            Serial.print(" d: ");
            Serial.print(derivative);
            Serial.print(" pid result: ");
            Serial.print(swPidResult);
            Serial.print(" tE: ");
            Serial.println(swKiTotalError);
          }
    
          // set motor speed variables for the Side-Facing PID results
          leftSpeed = MOTOR_BASE_SPEED;
          rightSpeed = MOTOR_BASE_SPEED;

          // add the result to the appropriate wheel
          leftSpeed = MOTOR_BASE_SPEED + swPidResult;
          rightSpeed = MOTOR_BASE_SPEED - swPidResult;
    
          // limits check, just to keep robot moving smoothly
          if(leftSpeed < MOTOR_MINIMUM_SPEED) {
            leftSpeed = MOTOR_MINIMUM_SPEED;
          }
          if(rightSpeed < MOTOR_MINIMUM_SPEED) {
            rightSpeed = MOTOR_MINIMUM_SPEED;
          }
        }
      }
      }

    // set motor speed for either PID by using leftSpeed/rightSpeed that were defined in either condition!
    if(MOVE_MOTORS) { 
      motors.setSpeeds(leftSpeed, rightSpeed);
    }
    if (MOTOR_DEBUG) {
      Serial.print("left: ");
      Serial.print(leftSpeed);
      Serial.print(" right: ");
      Serial.println(rightSpeed);
    }
    // reset PID flag (turn back on after US readings)
    pidFlag = false;
  }
}
