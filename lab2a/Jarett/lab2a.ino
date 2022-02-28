#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
Encoders encoders;

// Jarett Sutula
// Due 2/20/2022
// Lab 2a Objectives:
// - add ultrasonic sensor to robot
// - successfully produce CM in serial monitor

// ultrasonic pins
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;

// max distance for ultrasonic (cm)
// seems to work up to ~350 before it gets confused?
// leave it at 300 for now.
const int MAX_DISTANCE = 300;

// ultrasonic timings
// period needs to be long enough to not mess up distance readings (+~20ms)
unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long US_PERIOD = 100;

// current US distance reading
int distance = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // program running confirmation.
  delay(1000);
  buzzer.play("c32");

}

void loop() {
  // put your main code here, to run repeatedly:
  usReadCM();
}

void usReadCM() {
  currentMillis = millis();
  if(currentMillis > previousMillis + US_PERIOD) {

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

    previousMillis = currentMillis;
  }
}
