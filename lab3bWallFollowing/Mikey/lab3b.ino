
#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;

Motors motors;

Servo headServo; // create servo object to control a servo

// switches
const boolean HEAD_DEBUG = true;

// Head Servo Timing
unsigned long headCm;
unsigned long headPm;
//move every half second
const unsigned long HEAD_MOVEMENT_PERIOD = 400;

// head servo constants
const int HEAD_SERVO_PIN = 21;
const int NUM_HEAD_POSITIONS = 2;
// the angles at which the head turns
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {180, 90};

// head servo data
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

//Motor Timing
unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 20;

// motor constants
const float MOTOR_BASE_SPEED = 50.0;
const int MOTOR_MIN_SPEED = 30;
// determine normalization factor based on MOTOR_BASE_SPEED
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

// wheel speed
int leftSpeed = 0;
int rightSpeed = 0;

// variables for Ultrasonic sensor reading

// Initialize Ultrasonic
const int ECHO_PIN = 22;
const int TRIG_PIN = 4;

// Ultrasonic Max Distance
const int MAX_DISTANCE = 30; //(200 cm / 2 meters)

// Ultrasonic Timing
unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long US_PERIOD = 200; // Time to wait for 1st US to activate

// current US distance reading
int distance = 0;

// desired x position
double desiredState = (double) 20.0;

// front pid constants
const double frontKp = 1;
const double frontKi = 0.5;
const double frontKd = 7;

double frontPreviousError = 0;

double frontKiTotal = 0.0;

// side pid constants
const double sideKp = 1;
const double sideKi = 0.05;
const double sideKd = 0.8;

double sidePreviousError = 0;

double sideKiTotal = 0.0;

// flag used for when the front pid takes over
bool frontpid = false;

// global pid constants
double error = 0.0;
double proportional = 0.0;
double integral = 0.0;
float derivative = 0.0;
float pidResult = 0.0;

void setup() {
  //set up motors
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  //move head
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  Serial.begin(57600);

  // intialize the head position to start
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(40);

  // start delay
  delay(3000);
  buzzer.play("c32");

}

void loop() {
  moveHead();
}

void pid(int distance, int servoAngle) {
  if (servoAngle == 180 && frontpid == false) {
    error = distance - desiredState;
    
    proportional = sideKp * error;
    sideKiTotal += error;
    
    integral = sideKi * sideKiTotal;
    
    derivative = sideKd * (error - sidePreviousError);
    
    sidePreviousError = error;

    pidResult = proportional + integral + derivative;

    leftSpeed = MOTOR_BASE_SPEED + pidResult;
    rightSpeed = MOTOR_BASE_SPEED - pidResult;

  } else if (servoAngle == 90 && distance < 20) {
    frontpid = true;
    
    error = distance - desiredState;

    proportional = frontKp * error;
    frontKiTotal += error;

    integral = frontKi * frontKiTotal;

    derivative = frontKd * (error - frontPreviousError);
    frontPreviousError = error;

    pidResult = proportional + integral + derivative;

    leftSpeed = MOTOR_BASE_SPEED + pidResult;
    rightSpeed = MOTOR_BASE_SPEED - pidResult;

  } else if (servoAngle == 90 && distance > 20) {
    frontpid = false;
  }

  motors.setSpeeds(leftSpeed, rightSpeed);
}

void moveHead() {

  headCm = millis();
  if (headCm > headPm + HEAD_MOVEMENT_PERIOD) {

    // position head to the current position in the array
    headServo.write(HEAD_POSITIONS[currentHeadPosition]);

    //set distance based on servo angle
    pid(distance, HEAD_POSITIONS[currentHeadPosition]);

    // take distance reading from ultrasonic sensor
    usReadCm();

    /**
       Set next head position
       Moves servo to the next head position and changes direction when needed.
    */
    if (headDirectionClockwise) {
      if (currentHeadPosition >= (NUM_HEAD_POSITIONS - 1)) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      }
      else {
        currentHeadPosition++;
      }
    }
    else {
      if (currentHeadPosition <= 0) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      }
      else {
        currentHeadPosition--;
      }
    }

    //reset previous millis
    headPm = headCm;
  }
}



void usReadCm() {
  currentMillis = millis();
  if (currentMillis > previousMillis + US_PERIOD) {

    // Clears the TRIG_PIN, (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read the ECHO_PIN, returns the sound wave travel time in microseconds
    // note the duration (38000 microseconds) that will allow for reading up max distance supported by the sensor
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Time of flight equation: Speed of sound wave divided by 2.

    // apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = 0;

    // Update the prevmillis
    previousMillis = currentMillis;
  }
}
