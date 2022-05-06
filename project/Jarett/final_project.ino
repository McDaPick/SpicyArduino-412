#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Servo headServo;
Motors motors;
Encoders encoders;

// Jarett Sutula
// Due 5/9/2022
// Final Project Objectives:
// - Full localization controller in 2D Cartesian Space
// - Move to multiple locations stored as x, y coordinates
// - hit said distances within 2cm accuracy
// - emit a note when it reaches a waypoint and a different one for the last destination
// - stop at last destination
// - avoid obstacles using potential fields

// Debugs for printing and flags
const boolean ANGLE_DEBUG = false;
const boolean PID_DEBUG = false;
const boolean MOTOR_DEBUG = false;
const boolean MOVE_MOTORS = true;
const boolean SL_SR_DEBUG = false;
const boolean UPDATED_POSITION_DEBUG = false;
const boolean GOAL_DEBUG = true;
const boolean MAGNITUDE_DEBUG = false;
const boolean HEAD_DEBUG = false;
const boolean INDIV_DISTANCE_DEBUG = false;
const boolean DISTANCE_ARRAY_DEBUG = false;
const boolean OBSTACLE_AVG_DEBUG = false;

// Localization Goals
const int NUMBER_OF_GOALS = 4;
float xGoals[NUMBER_OF_GOALS] = {40, 0, -40, 0};
float yGoals[NUMBER_OF_GOALS] = {0, 0, 0, 0};
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

// Servo Constants and Timing
const int HEAD_SERVO_PIN = 20;
unsigned long headServoCm;
unsigned long headServoPm;
const unsigned long HEAD_SERVO_MOVEMENT_PERIOD = 150;
const int NUM_HEAD_POSITIONS = 5;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {130, 110, 90, 70, 50};
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;
// Servo rotation is ~-7 degrees off. so for a true 'straight 90 degree' angle, we need 83.
// saving the buffer here lets us change it if we use a different servo or anything.
const int HEAD_ROTATION_BUFFER = -7;
// Need to give the servo time to move before we measure the distance!
const unsigned long US_WAITING_TIME = 80;

// Ultrasonic Constants and Timing
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;
const float MAX_DISTANCE = 20.0;
float distance = 0;
boolean usReadFlag = false;
boolean obstacleDetection = false;
unsigned long usCm;
unsigned long usPm;

// fill array with max_distance so it doesn't react to anything at the start.
float distanceArray[NUM_HEAD_POSITIONS] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};
// array of proportional response for servo positions.
// more focus on angles closer to front! higher proportional!
const float magnitudeArray[NUM_HEAD_POSITIONS] = {1.0, 1.5, 2.0, 1.5, 1.0};
float magnitudeFactor = 0.0;
float obstacleFactor = 0.0;
float pidResult = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  // reverse those motors since our guy is backwards!
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  //encoders.flipEncoders(true);

  //servo setup
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(HEAD_POSITIONS[0]+HEAD_ROTATION_BUFFER);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

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
    moveHead();
    usReadCm();
    updatePosition();
    pidCalculations();
    obstacleCalculations();
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
  countLeft += encoders.getCountsAndResetRight();
  countRight += encoders.getCountsAndResetLeft();

  // update Sl and Sr values
  Sl = ((countLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
  Sr = ((countRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

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

  // apply blanket atan2 call to fix ~180 turn wiggle bug
  error = atan2(sin(error), cos(error));
  
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
  pidResult = proportional + integral + derivative;

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
  magnitudeFactor = new_mag_2;
  
  if(MAGNITUDE_DEBUG) {
    Serial.print(" X: ");
    Serial.print(p[X_POSITION]);
    Serial.print(" Y: ");
    Serial.print(p[Y_POSITION]);
    Serial.print(" magnitude: ");
    Serial.println(new_mag_2);
  }
}

void moveMotors() {
  // Update leftSpeed/rightSpeed with pid result and obstacle factor
  leftSpeed = MOTOR_BASE_SPEED * magnitudeFactor + pidResult - obstacleFactor;
  rightSpeed = MOTOR_BASE_SPEED * magnitudeFactor - pidResult + obstacleFactor;
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
//    Serial.print("L: ");
//    Serial.print(leftSpeed);
//    Serial.print(" R: ");
//    Serial.print(rightSpeed);
//    Serial.print(" pid: ");
//    Serial.print(pidResult);
//    Serial.print(" obsFac: ");
//    Serial.println(obstacleFactor);
}

void moveHead() {
  headServoCm = millis();
  if(headServoCm > headServoPm + HEAD_SERVO_MOVEMENT_PERIOD) {
    if(HEAD_DEBUG) {
      Serial.print("position: ");
      Serial.print(currentHeadPosition);
      Serial.print(" degrees: ");
      Serial.println(HEAD_POSITIONS[currentHeadPosition]);
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

    // position head to the current position index.
    headServo.write(HEAD_POSITIONS[currentHeadPosition] + HEAD_ROTATION_BUFFER);
    
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
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
    // divide by 2 from the total distance the sound has traveled
    // we only care about the one-way trip to the object!
    distance = duration * 0.034 / 2;
  
    // apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = MAX_DISTANCE;

    // set appropriate array value to reading based on position
    distanceArray[currentHeadPosition] = distance;
  
    // serial monitor output
    if(INDIV_DISTANCE_DEBUG) {
      Serial.print(" distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }

    // print out last recorded distances for each angle
    if(DISTANCE_ARRAY_DEBUG) {
      Serial.print(" [");
      for(int i = 0; i < NUM_HEAD_POSITIONS; i++) {
        Serial.print(distanceArray[i]);
        if(i != NUM_HEAD_POSITIONS - 1) {
          Serial.print(", ");
        }
      }
      Serial.print("] ");
    }
    // reset flag so it knows not to run this code until the flag
    // is set back to false when servo is changing angle in moveHead().
    usReadFlag = true;
    // reset flag so it knows to run obstacle detection after getting
    // updated readings.
    obstacleDetection = true;
  }
}

void obstacleCalculations() {
  if(obstacleDetection) {
    // get 'half' averages from left [0, 1] and right [3, 4] positions.
    float leftAvg = 0.0;
    float rightAvg = 0.0;
    float midDistance = distanceArray[3];
    
    // need closer reading = MORE reaction. do this by subtracting from max distance.
    for(int i = 0; i < 2; i++) {
      leftAvg += (MAX_DISTANCE - distanceArray[i]) * magnitudeArray[i];
    }
    for(int i = 3; i < 5; i++) {
      rightAvg += (MAX_DISTANCE - distanceArray[i]) * magnitudeArray[i];
    }
    
    leftAvg /= 2;
    rightAvg /= 2;
    
    if(OBSTACLE_AVG_DEBUG) {
      Serial.print("leftAvg: ");
      Serial.print(leftAvg);
      Serial.print(" rightAvg: ");
      Serial.println(rightAvg);
      Serial.print(" X: ");
      Serial.print(p[X_POSITION]);
      Serial.print(" Y: ");
      Serial.print(p[Y_POSITION]);
    }
    obstacleDetection = false;

    // apply leftAvg and rightAvg to appropriate speeds
    // if left > right, then left needs to be + to push it away from obstacle.
    // so add factor to leftSpeed, subtract factor from rightSpeed
    // also need to make sure the middle value pushes to the correct side based
    // on leftAvg and rightAvg. If they are the same, turn right as default.
    float midFactor = (MAX_DISTANCE - midDistance) * magnitudeArray[2];
    if (leftAvg < rightAvg) {
      midFactor *= -1.0;
    }
    // give the factor that will change speeds in moveMotors()
    obstacleFactor = leftAvg - rightAvg + midFactor;
    
    // TODO: check to see if they are != 0 and still equal? might need to 
    // guide it in a specific direction?
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
