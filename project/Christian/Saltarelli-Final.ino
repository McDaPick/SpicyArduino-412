// [Final] Localization + Object Awarness
//   - This program is a full localization controller in 2D cartesian space. With the added
//   capability to recognize objects within its direct environment. Upon being given a respective
//   goal. This program enables the ability to reach that goal whilst avoiding objects inhibiting 
//   its path. 
//
// @author Christian Saltarelli
// @date 4-29-22
#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Servo servo;
Motors motors;
Buzzer buzzer;

// Switches
const boolean EN_DEBUG = false;
const boolean US_DEBUG = true;
const boolean HEAD_DEBUG = false;
const boolean LOCALE_DEBUG = false;
const boolean ERROR_DEBUG = false;
const boolean MOTOR_DEBUG = false;

const boolean MOTOR_ON = false;
const boolean US_ON = true;

// Goal Constraints
const int NUM_GOALS = 1;
float Goals[NUM_GOALS][2] = {{80.0, 50.0}};
int currentGoal = 0; // Default
boolean completed = false;

// Localization Properties
float pose[3] = {0.0, 0.0, 0.0}; // Default Position [X, Y, Theta]
const float b = 8.574; 

float deltaS = 0;
float deltaTheta = 0;
float deltaX = 0;
float deltaY = 0;

// Servo Properties
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_PERIOD = 150;

const int SERVO_PIN = 20;
const int NUM_POSITIONS = 5;
const int SERVO_POSITIONS[NUM_POSITIONS] = {135, 120, 90, 60, 45};
const float SERVO_PRIORITY[NUM_POSITIONS] = {1.0, 1.5, 2.0, 1.5, 1.0};

boolean servoDirectionClockwise = true;
int currentHeadPosition = 2; // Index

// Ultrasonic Properties
const int ECHO_PIN = 12;
const int TRIG_PIN = 18;

const float MAX_DISTANCE = 20.0;
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
float distances[NUM_POSITIONS] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};

unsigned long usCm;
const unsigned long US_PERIOD = 80;   // Interval to Check US
boolean usReadFlag = false;           // Ensures 1 reading from US

// Encoder Conditions
unsigned long enCM;
unsigned long enPM;
const unsigned long EN_PERIOD = 30;

long countsLeft = 0;
long countsRight = 0;
long prevRight = 0;
long prevLeft = 0;

// Distance Calculation Properties
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0351;

float Sl = 0.0F;
float Sr = 0.0F;

float distance_factor = sqrt(sq(Goals[currentGoal][0] - pose[0]) + sq(pose[1] - Goals[currentGoal][1]));

// PID Constants
const float kp = 52;
const float ki = 0.0;
const float kd = 0.0;

float runningTotal = 0;
float errorRate = 0;
float prevErrorRate = 0; 

// Speed + Motor Factor Properties
const float BASE_SPEED = 90.0;
const float MOTOR_FACTOR = BASE_SPEED / 100;

void setup() {
  // Configure Serial Monitor
  Serial.begin(57600);

  // Update Pin Mapping + Servo
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  servo.attach(SERVO_PIN);
  servo.write(SERVO_POSITIONS[2]); // Default 0
  
  // Configure Motors + Encoders
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  
  encoders.flipEncoders(true);

  // Notify Program Start
  buzzer.play("c32");
  delay(1000);
}

void loop() {
  if (!completed) {
    // Check Current Goal
    checkGoal();

    // Update Robot Mapping + US Position
    updateState();
    
  } else {
    // Ensure Robot isn't moving
    motors.setSpeeds(0.0, 0.0);
  }

}

/**
 * updateState()
 *  - Encapsulates updating our internal
 *  mapping for both Localization +
 *  Object Detection.
 */
void updateState() {
  // Collect Readings for Encoders + US
  enReadCM();
  usReadCM();

  // Get Goal + Obj Result
  float goalResult = updatePID();
  float objResult = updateDetection();
  
  // Update Motors w/ Results
  setMotors(goalResult, objResult);

}

/**
 * checkGoal()
 *  - Utilized to check if we have 
 *  reached our current goal.
 */
void checkGoal() {
  // Check if Current Goal has been Reached
  if ((Goals[currentGoal][0] - 0.5) <= pose[0] && pose[0] <= (Goals[currentGoal][0] + 0.5)
  && (Goals[currentGoal][1] - 0.5) <= pose[1] && pose[1] <= (Goals[currentGoal][1] + 0.5)) {
    // Notify Completion of Goal
    buzzer.play("d32");

    // Proceed to Next Goal
    if (currentGoal < NUM_GOALS - 1) {
      currentGoal++;

      // Get New Distance Factor
      distance_factor = getDistanceFactor();
      
    } else if (currentGoal == NUM_GOALS - 1) {
      // Update Completed Flag
      completed = true;

      motors.setSpeeds(0.0, 0.0);

      // Signal Completion
      buzzer.play("c32");
      delay(500);
      buzzer.play("d32");
      delay(500);
      buzzer.play("c32");
    } 
  } 
}

/**
 * updatePosition()
 *  - The primary driver for our Localization logic
 *  starts by evaluating our distance traveled, then 
 *  results calculated are used to update our properties.
 */
void enReadCM() {
  enCM = millis();

  if (enCM > enPM + EN_PERIOD) {
    // Get Current Total Clicks 
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // Update Distance Traveled for current period
    Sl = ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr = ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    // Calculate our Change in S (Center Point)
    deltaS = (Sr + Sl) / 2;

    if (EN_DEBUG) {
      Serial.print("Distance Sl (Left): ");
      Serial.println(Sl);
      Serial.print("Distance Sr (Right): ");
      Serial.println(Sr);
      Serial.print("Change in S: ");
      Serial.println(deltaS);
    }

    // Update Prev Millis
    enPM = enCM;

    // Update Previous Tick Counts
    prevLeft = countsLeft;
    prevRight = countsRight;

    // Update Localization
    updateLocale();

  }

  // Update Head
  setHead();
}

/**
 * usReadCm()
 * - Used to initiate our US and
 *   determine the respective distance (cm)
 *   from any returned sound waves.
 */
void usReadCM() {
  usCm = millis();

  if (usCm > headPm + US_PERIOD && !usReadFlag) {
    if (US_ON) {
      // Clear TRIG_PIN
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);

      // Initiate our TRIG_PIN to HIGH (active) for 10s
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      // Get Sound ToF via ECHO_PIN
      long duration = pulseIn(ECHO_PIN, HIGH, 30000);

      // Calculate Distance w/ ToF for Respective Head Position
      distances[currentHeadPosition] = duration * 0.034 / 2;

      // Apply Respective Limits
      if (distances[currentHeadPosition] > MAX_DISTANCE) distances[currentHeadPosition] = MAX_DISTANCE;
      if (distances[currentHeadPosition] == 0) distances[currentHeadPosition] = MAX_DISTANCE;

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
    }
  }

  // Update Head Position for Next Reading
//  setHead();
}

void setHead() {
  headCm = millis();

  if (headCm > headPm + HEAD_PERIOD) {
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

    // Debug Flag Output
    if (HEAD_DEBUG) {
      Serial.print("Angle: ");
      Serial.print(currentHeadPosition);
      Serial.print(" - ");
      Serial.println(SERVO_POSITIONS[currentHeadPosition]);
      Serial.println("################");
    }


    // Update Previous Millis
    headPm = headCm;

    // Update US Read Flag
    usReadFlag = false;
  }
}


/**
 * updateLocale()
 *  - Updates our pose based on the
 *  calculated change of S from reading
 *  our encoders.
 */
void updateLocale() {
  // Calculate our change in Theta
  deltaTheta = (Sr - Sl) / b;

  // Calculate our change in X + Y
  deltaX = deltaS * cos(pose[2] + (deltaTheta/2));
  deltaY = deltaS * sin(pose[2] + (deltaTheta/2));

  // Apply Changes to Pose
  pose[0] += deltaX;
  pose[1] += deltaY;
  pose[2] += deltaTheta;

  if (LOCALE_DEBUG) {
    Serial.print("  Current X: ");
    Serial.print(pose[0]);
    Serial.print("  Current Y: ");
    Serial.print(pose[1]);
    Serial.print("  Current Theta: ");
    Serial.print(pose[2]);
    Serial.println();
  }
}

/**
 * updateDetection()
 *  - Updates our expected reaction
 *  to the detection of an object
 *  within our environment.
 */
float updateDetection() {
  // TODO:- Implement Detection Functionality
  return 0.0;
}

/**
 * updatePID()
 *  - Used to calculate our error rate
 *  based on the components of the PID 
 *  algorithm.
 */
float updatePID() {
  // Compute Rate of Error (Radians)
  double desiredTheta = atan2((Goals[currentGoal][1] - pose[1]), (Goals[currentGoal][0] - pose[0]));
  errorRate = desiredTheta - pose[2];

  if (ERROR_DEBUG) { 
    Serial.print("Desired Theta: ");
    Serial.print(desiredTheta);
    Serial.print("  Current Theta: ");
    Serial.println(pose[2]);
  }

  // Calculate Proportional [Kp * e]
  float proportional = kp * errorRate;

  // Calculate Integral Correction [Ki * errorSum] + Update RunningTotal
  runningTotal += errorRate;
  float integral = ki * runningTotal;

  // Calculate Derivative [Kd * (e - prev_e)]
  float derivative = kd * (errorRate - prevErrorRate);

  // Update Previous Error Rate
  prevErrorRate = errorRate;

  // Return Calculated Result
  return (proportional + integral + derivative);
}

/**
 * setMotors()
 *  - Updates our Motors from the result
 *  of our PID algorithm.
 */
void setMotors(float gResult, float oResult) {  
  // Start with BASE_SPEED
  float leftSpeed = BASE_SPEED;
  float rightSpeed = BASE_SPEED;

  // Calculate Distance + Magnitude
  float current_distance = sqrt(sq(Goals[currentGoal][0] - pose[0]) + sq(pose[1] - Goals[currentGoal][1]));
  float magnitude = dampen((float)(current_distance) / distance_factor);
  
  // Calculate Speed based on PID Result + Magnitude
  leftSpeed -= (gResult * magnitude);
  rightSpeed += (gResult * magnitude);

  if (MOTOR_DEBUG) {
    Serial.print("PID Result: ");
    Serial.print(gResult);
    Serial.print("  Distance Factor: ");
    Serial.print(distance_factor);
    Serial.print("  Magnitude: ");
    Serial.print(magnitude);
    Serial.print("  Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print("  Right Speed: ");
    Serial.print(rightSpeed);
    Serial.println();
    Serial.println("#######################");
    Serial.println();
  }

  if (MOTOR_ON) {
    // Update Motor Speeds
    motors.setSpeeds(leftSpeed, rightSpeed);
  }
}

/**
 * getDistanceFactor()
 * - Utility function for calculating
 *   our respective distance factor at
 *   a given point in time.
 */
float getDistanceFactor() {
  return sqrt(sq(Goals[currentGoal][0] - pose[0]) + sq(pose[1] - Goals[currentGoal][1]));
}

/**
 * dampen(value)
 * - Utility function for dampening our
 *   magnitude.
 */
float dampen(float value) {
  return 0.2 * sin(PI * value) + 0.8;
}
