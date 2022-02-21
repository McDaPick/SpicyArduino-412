#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer; 
Motors motors;

//Initialize Ultrasonic
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;

//Ultrasonic Max Distance
const float MAX_DISTANCE = 100.0; //(200 cm / 2 meters)

// determine the nomalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 5;

// motor constants
const float MOTOR_BASE_SPEED = 300.0;
const int MOTOR_MIN_SPEED = 30;
// determine normalization factor based on MOTOR_BASE_SPEED
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;


//Ultrasonic Timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50; // Time to wait for 1st US to activate

//Motor Timing
unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 20;

// current US distance reading
int distance = 0;

void setup() {
// put your setup code here, to run once:
  
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  delay(1000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:

  // update the current distance
  usReadCm();

  // update the motor speed
  setMotors();
}

void usReadCm(){
  usCm = millis();
  if(usCm > usPm + US_PERIOD) {
    
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
    if(distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if(distance == 0) distance = MAX_DISTANCE;

    // Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Update the prevmillis
    usPm = usCm;
  }
}

  void setMotors(){
  motorCm = millis();
  if(motorCm > motorPm + MOTOR_PERIOD) {

    // start out with the MOTOR_BASE_SPEED
    float leftSpeed = MOTOR_BASE_SPEED;
    float rightSpeed = MOTOR_BASE_SPEED;

    // check to see if most current distance measurement is less than / equal to MAX_DISTANCE
    if(distance <= MAX_DISTANCE){

      // determine the magnitude of the distance by taking the difference (short distance = high magnitude)
      // divide by the DISTANCE_FACTOR to ensure uniform response as MAX_DISTANCE changes
      // This maps the distance range (1 - MAX_RANGE) to 0-100 for the magnitude
      float magnitude = (float)(MAX_DISTANCE - distance) / DISTANCE_FACTOR;
      // example 1: MAX_DISTANCE = 80, distance = 40, 80-40= 40/.8 = 50 (mid range!)
      // example 2: MAX_DISTANCE = 160, distance = 40, 160-40= 120/1.6 = 75 (top 1/4)

      // Multiple the magnitude by the MOTOR_FACTOR to map the magnitude range (0-100) of the motors
      // (0- MOTOR_BASE_SPEED)
      leftSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
      rightSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
    }

    // lower limit check
    if(leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
    if(rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

    // check stop distance
    if(distance <= STOP_DISTANCE) leftSpeed = 0;
    if(distance <= STOP_DISTANCE) rightSpeed = 0;

    Serial.print("Left: ");
    Serial.print(leftSpeed);
    Serial.print(" Right: ");
    Serial.println(rightSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed);

    motorPm = motorCm;  
    }
  }
