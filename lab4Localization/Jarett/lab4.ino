#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Servo headServo;
Motors motors;
Encoders encoders;

// Jarett Sutula
// Due 4/18/2022
// Lab 4 Objectives:
// - Full localization controller in 2D Cartesian Space
// - Move to multiple locations stored as x, y coordinates
// - hit said distances within 2cm accuracy
// - emit a note when it reaches a waypoint and a different one for the last destination
// - stop at last destination

// Debugs for printing and flags
const boolean ANGLE_DEBUG = false;
const boolean PID_DEBUG = false;
const boolean MOTOR_DEBUG = false;
const boolean MOVE_MOTORS = true;
const boolean SL_SR_DEBUG = false;
const boolean UPDATED_POSITION_DEBUG = false;
const boolean GOAL_DEBUG = false;
const boolean MAGNITUDE_DEBUG = true;

// Localization Goals
const int NUMBER_OF_GOALS = 4;
float xGoals[NUMBER_OF_GOALS] = {80, -60, -60, 0};
float yGoals[NUMBER_OF_GOALS] = {50, 0, -30, 0};
int current_goal = 0;
// should we keep moving towards a goal?
boolean notDone = true;

// Position in space
// p = [x, y, theta]
const int X_POSITION = 0;
const int Y_POSITION = 1;
const int THETA_POSITION = 2;
const int NUMBER_OF_POSITION_VARS = 3;
// actual location of robot's current position
float p[NUMBER_OF_POSITION_VARS] = {0, 0, 0};
// constant value of distance between drive wheels (8.574 cm)
const float b = 8.574;
  
// PID Constants and Variables
const double kp = 25;
const double ki = 0.5;
const double kd = 0;
double kiTotalError = 0.0;
double previousError = 0.0;
// unsure if we will need it, but totalerror limit
const double integralLimit = 5;

// keep track of distance encoders.
long countLeft = 0;
long prevLeft = 0;
long countRight = 0;
long prevRight = 0;

// keep track of wheel movement and specs
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.0531;
float Sl = 0.0F;
float Sr = 0.0F;

// Motor Control
const double MOTOR_BASE_SPEED = 70;
const double MOTOR_MINIMUM_SPEED = 40;
float leftSpeed = MOTOR_BASE_SPEED;
float rightSpeed = MOTOR_BASE_SPEED;
float distance_factor = sqrt(sq(xGoals[current_goal] - p[X_POSITION]) + sq(yGoals[current_goal] - p[Y_POSITION]));

// Timing Constants
unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long PERIOD = 20;

// Song notes
const int gs5 = 831;
const int as5 = 932;
const int c6 = 1047;
boolean playedSong = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  // reverse those motors since our guy is backwards!
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  // small delay to prep
  delay(3000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  localization();
}

void localization() {
  // run all of the relevant code only after the designated ms delay
  currentMillis = millis();
  if (currentMillis > previousMillis + PERIOD && notDone) {
    checkGoals();
    checkEncoders();
    updatePosition();
    pidCalculations();
    moveMotors();
    // reset delay timer
    previousMillis = currentMillis;
  }
  if(!notDone && !playedSong) {
    playSong();
  }
}

void checkGoals() {
  // compare current X, Y to the current_goal X, Y positions.
  if(p[X_POSITION] >= xGoals[current_goal] - 0.5 && p[X_POSITION] <= xGoals[current_goal] + 0.5 &&
  p[Y_POSITION] >= yGoals[current_goal] - 0.5 && p[Y_POSITION] <= yGoals[current_goal] + 0.5) {
    // if we are not at our final destination (current_goal = 2), play waypoint sound and increment
    if(current_goal != NUMBER_OF_GOALS - 1) {
      buzzer.play("c32");
      current_goal++;
      Serial.println("moving to next goal!");
      distance_factor = sqrt(sq(xGoals[current_goal] - p[X_POSITION]) + sq(yGoals[current_goal] - p[Y_POSITION]));
    } else {
      // play destination sound and change notDone flag
      motors.setSpeeds(0,0);
      notDone = false;
    }
  }
}

void checkEncoders() {
  // get encoder values for both sides
  // THESE NEED TO BE FLIPPED TO GET CORRECT Y VALUES
  // flipping motors does not flip encoders. deltaTheta relies on these reflecting
  // the robot's left and right wheels according as Sl and Sr, so we can keep the equations
  // the same, but since the robot's 'left' wheel in odometry is actually its technical 'right'
  // wheel, we need to swap them here to make sure our flipped motors encode each side accurately.
  countLeft += encoders.getCountsAndResetRight();
  countRight += encoders.getCountsAndResetLeft();

  // update Sl and Sr values
  Sl = ((countLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
  Sr = ((countRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

  // Sl and Sr DO NOT FLIP with the motors flip code.
  // need to inverse them manually.
  Sl *= -1.0;
  Sr *= -1.0;

  if(SL_SR_DEBUG) {
    Serial.print("Sl: ");
    Serial.print(Sl);
    Serial.print(" Sr: ");
    Serial.println(Sr);
  }

  // update prevLeft and prevRight for next loop
  prevLeft = countLeft;
  prevRight = countRight;
}

void updatePosition() {
  // determine change in x, change in y, and change in theta to update position
  // change in middle-point between wheels
  float deltaS = (Sr + Sl) / 2.0;

  // change in theta 
  float deltaTheta = (Sr - Sl) / b;

  // change in X and Y coordinates in space
  float deltaX = deltaS * cos(p[THETA_POSITION] + (deltaTheta / 2));
  float deltaY = deltaS * sin(p[THETA_POSITION] + (deltaTheta / 2));

  // update position of robot now that we know how much we've changed
  p[X_POSITION] += deltaX;
  p[Y_POSITION] += deltaY;
  p[THETA_POSITION] += deltaTheta;

  if(UPDATED_POSITION_DEBUG) {
    Serial.print("x: ");
    Serial.print(p[X_POSITION]);
    Serial.print(" y: ");
    Serial.print(p[Y_POSITION]);
    Serial.print(" theta: ");
    Serial.println(p[THETA_POSITION]);
  }
  
}

void pidCalculations() {
  // get the error based on difference between current position and current goal.
  float yDifference = yGoals[current_goal] - p[Y_POSITION];
  float xDifference = xGoals[current_goal] - p[X_POSITION];

  // using atan2 to get radian amount for needed angle from current pos.
  double desiredTheta = atan2(yDifference, xDifference);

  // Calculate error based on difference between current and desired theta.
  // This means that if our robot is on the correct angle to the goal, difference
  // will be 0 - meaning error = 0 and the robot stays straight!
  // positive error = needs to turn left to increase theta
  // negative error = needs to turn right to decrease theta
  double error = desiredTheta - p[THETA_POSITION];
  
  // Calculate Proportional part of PID
  double proportional = kp * error;

  // add error to Integral part of PID
  kiTotalError += error;

  // Apply limits to Ki if needed.
  if (kiTotalError > integralLimit) {
    kiTotalError = integralLimit;
  } else if (kiTotalError < integralLimit * -1) {
    // also apply to the negative side as well
    kiTotalError = integralLimit * -1;
  }

  // Calculate Integral part of PID
  double integral = ki * kiTotalError;

  // Calculate Derivative part of PID
  float derivative = kd * (error - previousError);
  // set previous error for next PID run
  previousError = error;

  // PID result from calculations
  float pidResult = proportional + integral + derivative;

  if(ANGLE_DEBUG) {
    Serial.print("current pos in degrees: ");
    Serial.print(p[THETA_POSITION] * (180 / PI));
    Serial.print("current error: ");
    Serial.print(error);
    Serial.print(" radians   error in degrees: ");
    Serial.println(error * (180 / PI)); 
  }

  if(PID_DEBUG){
    Serial.print("error: ");
    Serial.print(error);
    Serial.print(" p: ");
    Serial.print(proportional);
    Serial.print(" i: ");
    Serial.print(integral);
    Serial.print(" d: ");
    Serial.print(derivative);
    Serial.print(" pid result: ");
    Serial.println(pidResult);
  }

  // shortened version to see error + location
  if(GOAL_DEBUG) {
    Serial.print("error: ");
    Serial.print(error);
    Serial.print(" dTheta: ");
    Serial.print(desiredTheta * (180/ PI));
    Serial.print(" X: ");
    Serial.print(p[X_POSITION]);
    Serial.print(" Y: ");
    Serial.print(p[Y_POSITION]);
    Serial.print(" theta: ");
    Serial.println(p[THETA_POSITION] * (180 / PI));
  }

  // calculate magnitude for speed
  // start slow and end slow - ramp up in between
  // we can use the current distance from goal to decide how to slow down.
  // sqrt(change in x ^2 + change in y ^2)
  float magnitude = sqrt(sq(xGoals[current_goal] - p[X_POSITION]) + sq(yGoals[current_goal] - p[Y_POSITION]));
  float new_mag = magnitude / distance_factor;
  float new_mag_2 = .2 * sin(PI*new_mag) + 0.8;
  
  
  if(MAGNITUDE_DEBUG) {
    Serial.print(" X: ");
    Serial.print(p[X_POSITION]);
    Serial.print(" Y: ");
    Serial.print(p[Y_POSITION]);
    Serial.print(" magnitude: ");
    Serial.println(new_mag_2);
  }
  // add the result to the appropriate wheel
  leftSpeed = MOTOR_BASE_SPEED * new_mag_2 + (pidResult);
  rightSpeed = MOTOR_BASE_SPEED * new_mag_2 - (pidResult);
}

void moveMotors() {
  // set motor speed for either PID by using leftSpeed/rightSpeed
    if(MOVE_MOTORS and notDone) { 
      motors.setSpeeds(leftSpeed, rightSpeed);
    }
    if (MOTOR_DEBUG) {
      Serial.print("left: ");
      Serial.print(leftSpeed);
      Serial.print(" right: ");
      Serial.println(rightSpeed);
    }
}

void playSong () {
  int volume = 10;
  buzzer.playFrequency(c6, 120, volume);
  delay(200);
  buzzer.playFrequency(c6, 120, volume);
  delay(200);
  buzzer.playFrequency(c6, 120, volume);
  delay(200);
  buzzer.playFrequency(c6, 120, volume);
  delay(400);
  buzzer.playFrequency(gs5, 120, volume);
  delay(400);
  buzzer.playFrequency(as5, 120, volume);
  delay(350);
  buzzer.playFrequency(c6, 120, volume);
  delay(250);
  buzzer.playFrequency(as5, 120, volume);
  delay(250);
  buzzer.playFrequency(c6, 120, volume);
  playedSong = true;
}
