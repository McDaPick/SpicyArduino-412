#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include <math.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Servo servo;
Motors motors;
Encoders encoders;

/**
 * Project
 *  
 * 
 * Created: 04/21/2022
 * Creator: Camerin Figueroa
 */

// https://pololu.github.io/pololu-3pi-plus-32u4-arduino-library/

/************************
 * Debug Variables      *
 ************************/
const bool MOTOR_DEBUG = false;
const bool PID_DEBUG = false;
const bool ODO_DEBUG = false;
const bool GOAL_DEBUG = false;
const bool US_DEBUG = false;
const bool SERVO_DEBUG = false;
const bool OBS_DEBUG = false;

/************************
 * PID Variables        *
 ************************/

// Desired Positions
const int totalPositions = 4;

// position: {x, y}

int positions[totalPositions][2] = {
  {100, 0}, // pos 1
  {0, 10}, // pos 2
  {50, 0}, // pos 3
  {0,0}}; // pos 4

int currPosition = 0;

// threshold for x/y pos before moving to next position
double posThreshold = 0.1; 

//Stores the current desired angle
double desiredAngle = 0;

// store pid results
double pidResult = 0;

// PID constants
// TODO: Tune constants
double kp = 0.0075;
double ki = 0.00012;
double kd = 0.000213;

// Limit PID from going off the rails
const double PID_MAX = 10000;
const double PID_MIN = -10000;
const double INTEGRAL_MAX = 1000;
const double INTEGRAL_MIN = -1000;

// Time between pid calculations
long timePassed = 0;

double kiTotal = 0.0; // summation of integral
double previousError = 0.0; // Keeps track of last error

// time between pid runs
long currTime = 0;
long prevTime = millis();

/************************
 * Motor Variables      *
 ************************/

const unsigned short MOTOR_PERIOD = 10;


// Motor Speed Factors
int MOTOR_MIN_SPEED = 20;
int MOTOR_BASE_SPEED = 50;
int MOTOR_MAX_SPEED = 75;

// temporarily stores motor base speed when slowing down
double CURR_BASE_SPEED;

// Controls when to run motor functions
unsigned long motorCm; // motor Current Millis
unsigned long motorPm; // motor Previous Millis

double slowdownThreshold = 5; // Distance till goal when we want to slowdown (cm)


// Stores motor speed
int leftSpeed = MOTOR_BASE_SPEED;
int rightSpeed = MOTOR_BASE_SPEED;

/************************
 * Servo Variables      *
 ************************/

// Init Servo Pin(s)
const short SERV_PIN = 21;
const short NUM_SERVO_POSITIONS = 5;
const short SERVO_POSITIONS[NUM_SERVO_POSITIONS]  = {140, 115, 90, 65, 40};
const short START_POS = SERVO_POSITIONS[0];

// PERIOD
int SERVO_PERIOD = 20;

// Timing Parameters
unsigned long servoCm=0; // Current measurement
unsigned long servoPm=0; // Previous Measurement

// Servo Data
boolean servoDirectionClockwise = true;
short currentServoPos = 0;

// Position Names

String posNames[5] = {"A","B","C","D","E"};

/************************
 * US Variables         *
 ************************/

// Initialize Ultrasonic
const short ECHO_PIN = 22;
const short TRIG_PIN = 4;

//Ultrasonic Maxs
const short MAX_DISTANCE = 50.0; //(200 cm/2m)

// Ultrasonic timing
unsigned long usCm; // Ultrasonic Current Millis
unsigned long usPm; // Ultrasonic Previous Millis
const unsigned short US_PERIOD = 50; // Time to wait btw checking the Ultrasonic Sensor

// Current US Reading in CM
double distance = 0; // Stores current distance temporarily
double distances[NUM_SERVO_POSITIONS] = {0.0, 0.0, 0.0, 0.0, 0.0}; // Record of distances recorded at each servo position#

// Keeps track of if distance was recorded yet
bool recordedDist = false;


/************************
 * Odometer Variables   *
 ************************/

// The current X/Y position
double currX = 0;
double currY = 0;

// The current angular direction (in radians) of the robot
// The robot will assume that it's initial direction is the one that is set below
double currentAngle = 0; //90*(M_PI/180);

double Sr = 0;
double Sl = 0;

// Used to control when to run odometer functions
double odoCm = 0; // current millis
double odoPm = 0; // previous millis

// Total left/right distance travelled
double totalLeft = 0;
double totalRight = 0;

// Click to Inch Conversions
int CLICKS_PER_ROTATION = 12;
double GEAR_RATIO = 75.81F;
double WHEEL_DIAMETER = 3.2; // 3.2 cm, 1.25984 in

double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*M_PI;

double CLICKS_PER_WHEELROTATION = (CLICKS_PER_ROTATION * GEAR_RATIO);

double DIST_PER_CLICK = WHEEL_CIRCUMFERENCE/CLICKS_PER_WHEELROTATION;

// Distance between both wheels
double DIST_BTW = 8.45;

/************************
 * Obstacle Avoidance   *
 ************************/ 

double obsFactor = 0; // Factor obstacle avoidance uses to affect motors
double obsRelianceRatio = 0; // How much should the obstacle avoidance affect motors vs the pid
double positionMag[NUM_SERVO_POSITIONS] = { 5, 10, 15, 10, 5};

/************************
 * Built In Functions   *
 ************************/ 

void setup(){

  // Setup Pins
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  servo.attach(SERV_PIN);

  // Set initial servo position
  servo.write(START_POS);

  // Flip motors since our robot goes in reverse
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  // Flip encoders since out robot goes in reverse
  encoders.flipEncoders(true);

  // Wait before starting
  buzzer.playNote(NOTE_C(4), 200, 10);
  delay(500);
  buzzer.playNote(NOTE_C(4), 200, 10);
  delay(500);
  buzzer.playNote(NOTE_C(4), 200, 10);
  delay(500);
  buzzer.playNote(NOTE_C(5), 400, 10);
  delay(1000);
  
  Serial.println("STARTING");

}

void loop() {
  // ** odometer and pid run within affectMotors **

  usReadcm();
  moveServo();
  
  affectMotors(); // Affect Motor Speeds
}

/**************************
 * Componential Functions *
 **************************/

void affectMotors(){
  /**
   * Motors and Speeds based off of calculated variables.
   * This includes using the pid calculations, distance calculations, and changing the goal position when necessary
   */
   
  motorCm = millis();
  if (motorCm > motorPm + MOTOR_PERIOD) {

    // Recalculate desiredAngle
    if (currPosition < totalPositions) {

      // Update location using odometer
      odometer();

      goalDetermine(); // Determines if we need to move to the next goal

      if (MOTOR_DEBUG) {
        Serial.print("X - dX  ");
        Serial.print(currX);
        Serial.print(" - ");
        Serial.print(positions[currPosition][0]);
        Serial.print("Y - dY  ");
        Serial.print(currY);
        Serial.print(" - ");
        Serial.print(positions[currPosition][1]);
      }

      desiredAngle = atan2(
        positions[currPosition][1]-currY,
        positions[currPosition][0]-currX);

      // Run PID
      pid();

      // RUN LOGIC TO GET LEFT/RIGHT SPEED

      // Determine the distance to the goal
      double distToGoal = sqrt(
        sq(positions[currPosition][0]-currX)+
        sq(positions[currPosition][1]-currY));
      
      if (distToGoal < slowdownThreshold) {
        CURR_BASE_SPEED = MOTOR_BASE_SPEED-((slowdownThreshold-distToGoal)*3);
      } else {
        CURR_BASE_SPEED = MOTOR_BASE_SPEED;
      }

      double leftSpeed = CURR_BASE_SPEED - pidResult - obsFactor;
      double rightSpeed = CURR_BASE_SPEED + pidResult + obsFactor;

      // Apply upper and lower limits to the left motor's speed
      if (leftSpeed > MOTOR_MAX_SPEED) {
        leftSpeed = MOTOR_MAX_SPEED;
      } else if (leftSpeed < MOTOR_MIN_SPEED) {
        leftSpeed = MOTOR_MIN_SPEED;
      }

      // Apply upper and lower limits to the right motor's speed
      if (rightSpeed > MOTOR_MAX_SPEED) {
        rightSpeed = MOTOR_MAX_SPEED;
      } else if (rightSpeed < MOTOR_MIN_SPEED) {
        rightSpeed = MOTOR_MIN_SPEED;
      }

      // Print debug info to serial
      if (MOTOR_DEBUG) {
        Serial.print(leftSpeed);
        Serial.print("|---|");
        Serial.println(rightSpeed);
        
      }
      // Set motor speeds
      motors.setSpeeds(rightSpeed,leftSpeed);
    } else {
      motors.setSpeeds(0,0);
    }
    
    // Reset motor previous millis
    motorPm = motorCm;
    
  }
}

void goalDetermine() {
  /**
   * goalDetermine - controls whether we've reached a goal and to move to the next goal
   **/
   
  // Increment desired pos if necessary
  if (abs(positions[currPosition][0]-currX) < posThreshold &&
      abs(positions[currPosition][1]-currY) < posThreshold) {
      /**
       * NEW POSITION
       **/
       
      if (GOAL_DEBUG) {
        // Print finished at position
        Serial.print("Finished at: (");
        Serial.print(currX);
        Serial.print(", ");
        Serial.print(currY);
        Serial.println(")");
        
        // Print new position
        Serial.print("New Goal: (");
        Serial.print(positions[currPosition][0]);
        Serial.print(", ");
        Serial.print(positions[currPosition][1]);
        Serial.println(")");
      }
      
      buzzer.playNote(NOTE_C(5), 400, 10); // Tell us that we've moved to next point
      
      currPosition++;

      // Pause before we go to next point
      motors.setSpeeds(0,0);
      delay(1000);
  }
}

void pid() {
  /**
   * PID - calculate pid to get to our goal
   **/

  // Get current time
  currTime = millis();

  if (PID_DEBUG) {
    Serial.print("Current Value -> Desired State");
    Serial.print(currentAngle);
    Serial.print(" -> ");
    Serial.println(desiredAngle);
  }

  timePassed += currTime-prevTime;
  
  prevTime = currTime;
  
  // Calculate Error

  double error = currentAngle - desiredAngle;
  
  error = atan2(sin(error), cos(error));

  if (PID_DEBUG) {
    Serial.print("ERROR: ");
    Serial.println(error);
    Serial.print("KP: ");
    Serial.println(kp);
    Serial.print("TP: ");
    Serial.println(timePassed);
  }

  // Calculate proportion
  double proportional = kp * error * timePassed;

  // Add current error to total error
  kiTotal += error;

  // Calculate the integral
  double integral = ki * kiTotal;

  // Limit Integral
  if (integral > INTEGRAL_MAX) {
    integral = INTEGRAL_MAX;
  } else if (integral < INTEGRAL_MIN) {
    integral = INTEGRAL_MIN;
  }

  // Calculate the Derivative
  double derivative = kd * (error - previousError);

  // Set previous error to current error
  previousError = error;

  // Final Pid
  pidResult = proportional + integral + derivative;

  // Limit PID
  if (pidResult > PID_MAX) {
    pidResult = PID_MAX;
  } else if(pidResult < PID_MIN) {
    pidResult = PID_MIN;
  }

  if (PID_DEBUG) {
    Serial.println(pidResult);
  }

}

void odometer() {
  /**
   * Odometer - calculated angle, and x/y position based on encoder readings
   **/
  // calculate Sr and Sl
  Sr = (encoders.getCountsAndResetRight()*DIST_PER_CLICK);
  Sl = (encoders.getCountsAndResetLeft()*DIST_PER_CLICK);

  double dist = (Sr+Sl)/2; // calculate distance moved
  double calcAngle = (Sr-Sl)/(DIST_BTW); // determine the radians rotated

  // Calculate x/y distance moved
  double x = dist * cos(currentAngle + (calcAngle/2));
  double y = dist * sin(currentAngle + (calcAngle/2));

  currentAngle += calcAngle; // increment current angle/theta

  // Increment current x/y values
  currX += x;
  currY += y;

  if (ODO_DEBUG) {
    Serial.print("Curr X: ");
    Serial.print(currX);
    Serial.print(" Curr Y: ");
    Serial.print(currY);
    Serial.print(" Curr Theta: ");
    Serial.println(currentAngle);
  }

}

void moveServo() {
  servoCm = millis();
  
  if (servoCm > servoPm + SERVO_PERIOD && recordedDist) {

    // debug output
    if(SERVO_DEBUG) {
      Serial.print(currentServoPos);
      Serial.print(" - ");
      Serial.print(SERVO_POSITIONS[currentServoPos]);
      Serial.print("->");
      Serial.println(servoDirectionClockwise);
    }
     
     recordedDist = false; // Reset so Servo doesn't move before recording new dist

     /**
     * Set next position
     * The servo will move to this position, next period
     */     

     // Reposition Servo
     if (servoDirectionClockwise) {
      if (currentServoPos >= (NUM_SERVO_POSITIONS - 1)) {
        
        // Reverse servo and decrement position number
        servoDirectionClockwise = !servoDirectionClockwise;
        currentServoPos = NUM_SERVO_POSITIONS-2;
        
      } else {
        // Increment servo position
        currentServoPos++;
      }
     } else {
      if (currentServoPos <= 0) {

        // Reverse Servo direction and increment servo position
        servoDirectionClockwise = !servoDirectionClockwise;
        currentServoPos = 1;
        
      } else {

        // Decrement Servo Position
        currentServoPos--;
        
      }
     }
     int pos = SERVO_POSITIONS[currentServoPos];
     
     servo.write(pos); // Move servo to new position
     usPm = millis()+75;

     // Reset servoPm
     servoPm = servoCm;

     obstacleDetect(); // Detect obstacles when head is moved
  }
}

void usReadcm() {  
  usCm = millis();
  if(usCm > usPm + US_PERIOD && !recordedDist) {
       // Clears the TRIG_PIN (set low)
       digitalWrite(TRIG_PIN, LOW);
       delayMicroseconds(2);
    
      // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      
      // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
      // note the duration (38000 microseconds) that will allow for reading up max
      long duration = pulseIn(ECHO_PIN, HIGH, 38000);
      
      // Calculating the distance
      distance = duration * 0.034 / 2; // TOF Equation
      
      // apply limits
      if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
      if (distance == 0) distance = MAX_DISTANCE;
      
      // Displays the distance on the Serial Monitor
      if (US_DEBUG){
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.print(" cm ");
        Serial.println(posNames[currentServoPos]);
      }
  
      // Store distance in distances array
      distances[currentServoPos] = distance;  
      
      recordedDist = true;
      
      // update the prevmillis
      usPm = usCm;
  }
}

void obstacleDetect() {
  /**
   * Detects obstacles and creates factors that the motors use to affect direction
   * 
   */


  double leftAvg=0;
  double rightAvg=0;
  double mid = 0;

  int halflen = NUM_SERVO_POSITIONS/2; // half of servopositions

  for (int i = 0; i < halflen; i++) {
    leftAvg += distances[i]*positionMag[i];
  }

  leftAvg /= halflen;
  
  for (int i = halflen+1; i<NUM_SERVO_POSITIONS; i++) {
    rightAvg += distances[i]*positionMag[i];
  }

  rightAvg /= halflen;

  mid = distances[halflen];
  
  if (mid < MAX_DISTANCE) {
    mid = MAX_DISTANCE-(mid * positionMag[halflen]);

    if (rightAvg <= leftAvg) {
      mid *= -1;
    }

  } else {
    mid = 0;
  }

  rightAvg = MAX_DISTANCE-rightAvg;

  leftAvg = MAX_DISTANCE-leftAvg;


  obsFactor = rightAvg - leftAvg + mid;


  /*
  if (rightAvg < leftAvg) {
    obsRelianceRatio = (1-(rightAvg/(MAX_DIST)))*2;
    //obsFactor = (90-SERVO_POSITIONS[closePos])*(M_PI/180)*obsRelianceRatio;
    obsFactor = ((90-SERVO_POSITIONS[NUM_SERVO_POSITIONS-1])*(M_PI/180)+currentAngle);
  } else if (leftAvg < MAX_DIST){
    obsRelianceRatio = (1-(leftAvg/(MAX_DIST)))*2;
    //obsFactor = (90-SERVO_POSITIONS[closePos])*(M_PI/180)*obsRelianceRatio;
    obsFactor = ((90-SERVO_POSITIONS[0])*(M_PI/180)+currentAngle);
  } else if (distances[3] < MAX_DIST) {
    obsRelianceRatio = (1-(distances[halflen]/(MAX_DIST)))*2;

    obsFactor = (90)*(M_PI/180);
  } else {
    obsFactor = 0;
    obsRelianceRatio = 0;
  }*/

  if (OBS_DEBUG) {
    Serial.print(leftAvg);
    Serial.print(" - ");
    Serial.print(obsFactor);
    Serial.print(" - ");
    Serial.println(rightAvg);
  }
}
