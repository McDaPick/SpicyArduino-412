// [Lab 4] Localization
//   - This program is a full localization controller in 2D cartesian space. This program enables
//   our robot to be given multiple locations stores as [x, y] coordinates and will be able 
//   to direct itself towards the goal locations given. 
//
// @author Christian Saltarelli
// @date 4-8-22
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Motors motors;
Buzzer buzzer;

// Switches
const boolean EN_DEBUG = false;
const boolean LOCALE_DEBUG = true;
const boolean ERROR_DEBUG = false;
const boolean MOTOR_DEBUG = false; 

const boolean MOTOR_ON = true;

// Goal Constraints
const int NUM_GOALS = 4;
float Goals[NUM_GOALS][2] = {{80, 50.0}, {-60.0, 0.0}, {-60, -30.0}, {0.0, 0.0}};
int currentGoal = 0; // Default
boolean completed = false;

// Localization Properties
float pose[3] = {0.0, 0.0, 0.0}; // Default Position [X, Y, Theta]
const float b = 8.574; 

float deltaS = 0;
float deltaTheta = 0;
float deltaX = 0;
float deltaY = 0;

// Encoder Conditions
unsigned long enCM;
unsigned long enPM;
const unsigned long EN_PERIOD = 30;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

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

  // Notify Program Start
  buzzer.play("c32");
  delay(1000);
}

void loop() {
  if (!completed) {
    // Check Current Goal
    checkGoal();
    
    // Update Robot Mapping
    enReadCM(); 
  } else {
    // Ensure Rbbot isn't moving
    motors.setSpeeds(0.0, 0.0);
  }
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

  // Update Motors based on PID Output
  setMotors(updatePID());
}

/**
 * updatePID()
 *  - Used to calculate our error rate
 *  based onn the components of the PID 
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
void setMotors(float result) {  
  // Start with BASE_SPEED
  float leftSpeed = BASE_SPEED;
  float rightSpeed = BASE_SPEED;

  // Calculate Distance + Magnitude
  float current_distance = sqrt(sq(Goals[currentGoal][0] - pose[0]) + sq(pose[1] - Goals[currentGoal][1]));
  float magnitude = dampen((float)(current_distance) / distance_factor);
  
  // Calculate Speed based on PID Result + Magnitude
  leftSpeed -= (result * magnitude);
  rightSpeed += (result * magnitude);

  if (MOTOR_DEBUG) {
    Serial.print("PID Result: ");
    Serial.print(result);
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
