#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
Encoders encoders;

// Jarett Sutula
// Due 2/27/2022
// Lab 2b Objectives:
// - object following behavior

// ultrasonic pins
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;

// max distance for ultrasonic (cm)
const float MAX_DISTANCE = 100.0;

// normalization for ultrasonic distance
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;

// how far the robot should stop away from an object.
const float STOP_DISTANCE = 15;

// current US distance reading
float distance = 0;

// ultrasonic timings
// period needs to be long enough to not mess up distance readings (+~20ms)
unsigned long usCurrentMillis;
unsigned long usPreviousMillis;
const unsigned long US_PERIOD = 50;

// motor timings
unsigned long motorCurrentMillis;
unsigned long motorPreviousMillis;
const unsigned long MOTOR_PERIOD = 20;

// motor constants
// on hardwood with current weight, robot needs AT LEAST 25.
const float MOTOR_BASE_SPEED = 300.0;
const int MOTOR_MIN_SPEED = 40;
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // program running confirmation.
  delay(1000);
  buzzer.play("c32");

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

}

void loop() {
  // put your main code here, to run repeatedly:
  usReadCM();
  setMotors();
}

void usReadCM() {
  // we can use the code from the previous lab to get the distance.
  // setMotors() will use the distance to move accordingly.
  usCurrentMillis = millis();
  if(usCurrentMillis > usPreviousMillis + US_PERIOD) {

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

    // apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = MAX_DISTANCE;

    // serial monitor output
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    usPreviousMillis = usCurrentMillis;
  }
}

void setMotors() {
  motorCurrentMillis = millis();
  if (motorCurrentMillis > motorPreviousMillis + MOTOR_PERIOD) {
    // start with the base speed
    float leftSpeed = MOTOR_BASE_SPEED;
    float rightSpeed = MOTOR_BASE_SPEED;

    // check if we are within our MAX_DISTANCE range.
    if (distance <= MAX_DISTANCE) {
      // determine the magnitude of the distance!
      float magnitude = (float)(MAX_DISTANCE - distance) / DISTANCE_FACTOR;

      // multiply the magnitude by the factor for the motors.
      leftSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
      rightSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
    }

    // lower limit check
    if(leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
    if(rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

    // check stopping distance - need to stop if we are within!
    if(distance <= STOP_DISTANCE) leftSpeed = 0;
    if(distance <= STOP_DISTANCE) rightSpeed = 0;

//    Serial.print("Left: ");
//    Serial.print(leftSpeed);
//    Serial.print(" Right: ");
//    Serial.println(rightSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed);

    motorPreviousMillis = motorCurrentMillis;
  }
}
