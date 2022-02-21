#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer; 
Motors motors;

//Initialize Ultrasonic
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;

//Ultrasonic Max Distance
const int MAX_DISTANCE = 200; //(200 cm / 2 meters)

//Ultrasonic Timing
unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long US_PERIOD = 100; // Time to wait for 1st US to activate

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
}

void usReadCm(){
  currentMillis = millis();
  if(currentMillis > previousMillis + US_PERIOD) {
    
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
    previousMillis = currentMillis;
  }
}
