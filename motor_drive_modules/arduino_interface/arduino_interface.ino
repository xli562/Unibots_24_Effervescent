/*  This file combines the interface of the following functionalities
    - arduino_intake_interface
    - ultrasound_arduino_interface

*/

//----------Intake----------
#define positivePin 12
#define negativePin 13

volatile unsigned long pulseHighTime = 0;
volatile unsigned long pulseLowTime = 0;
volatile unsigned long lastRiseTime = 0;
volatile bool isHigh = false;
int dutyCycle = 0;
int maxDutyCycle = 1100;
int minDutyCycle = 242;
int power = 0;
//----------------------------

//----------Ultrasound----------
#include <NewPing.h>

#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define NUM_ULTRASOUND 5

// int trigPins[NUM_ULTRASOUND] = {6,2,8,4,10};  //Right_Bottom, Left, Front, Back, Right_Top
// int echoPins[NUM_ULTRASOUND] = {7,3,9,5,11}; 
int trigPins[NUM_ULTRASOUND] = {4,A0,8,6,10};  //Right_Bottom, Left, Front, Back, Right_Top
int echoPins[NUM_ULTRASOUND] = {5,3,9,7,11};

NewPing ultrasound_sensors[NUM_ULTRASOUND]{
  NewPing(trigPins[0], echoPins[0], MAX_DISTANCE),
  NewPing(trigPins[1], echoPins[1], MAX_DISTANCE),
  NewPing(trigPins[2], echoPins[2], MAX_DISTANCE),
  NewPing(trigPins[3], echoPins[3], MAX_DISTANCE),
  NewPing(trigPins[4], echoPins[4], MAX_DISTANCE)
};
//----------------------------

void setup() {
  //----------Ultrasound----------
  Serial.begin(115200); // Initialize serial communication.

  Serial.println("!Restart!");

  //----------Intake----------
  //   Serial.begin(9600);
  pinMode(2, INPUT); // Using digital pin 2 for PWM input //NOTE: this pin is used by other one of the ultrasound
  pinMode(positivePin, OUTPUT);
  pinMode(negativePin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), measurePulse, CHANGE);
}

void loop() {

  //------------------------------Ultrasound-------------------------
  for (int i = 0; i < NUM_ULTRASOUND; i++) {
    // Read ultrasonic sensor distance
    delay(35);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    unsigned int ultrasonic_distance = ultrasound_sensors[i].ping_cm(); // Send ping, get distance in centimeters and print result (0 = outside set distance range)
  
    // Send data over serial with identifiers
    Serial.print("U"); // Identifier for ultrasonic sensor
    Serial.print(i); // Add index of the sensor for identification
    Serial.print(":");
    Serial.print(ultrasonic_distance);
    Serial.print(",");
  }
  Serial.println("");
  //-----------------------------------------------------------------

  //------------------------------Intake----------------------------
  if (pulseHighTime > 0 && pulseLowTime > 0) {
    unsigned long totalPulseTime = pulseHighTime + pulseLowTime;
    dutyCycle = (float)pulseHighTime / totalPulseTime * 10000.0;
    // Serial.print("Duty Cycle: ");
    // Serial.println(dutyCycle);

    // Reset for the next measurement
    pulseHighTime = 0;
    pulseLowTime = 0;
  }
  // Drive motors accordingly
  // Serial.print(dutyCycle);
  // Serial.print(" ");
  power = map(dutyCycle, minDutyCycle, maxDutyCycle, -255, 255);
  // Truncate power to [-1023, 1023]
  power = max(-255, min(power, 255));
  setPower(power);
  
  //-----------------------------------------------------------------

}

//----------Intake----------
void measurePulse() {
  if (digitalRead(2) == HIGH) {
    // Rising edge detected
    if (isHigh == false) {
      isHigh = true;
      unsigned long currentTime = micros();
      if (lastRiseTime > 0) {
        pulseLowTime = currentTime - lastRiseTime;
      }
      lastRiseTime = currentTime;
    }
  } else {
    // Falling edge detected
    if (isHigh == true) {
      isHigh = false;
      unsigned long currentTime = micros();
      pulseHighTime = currentTime - lastRiseTime;
    }
  }
}

//----------Intake----------
void setPower(int pwr) {
  bool eat = 0;
  if (abs(pwr) < 140) {
    // This little power will not turn the motor.
    // Thus setting outputs to 0
    analogWrite(negativePin, 0);
    analogWrite(positivePin, 0);
    // Serial.print(0);
    // Serial.print(", ");
    // Serial.println(0);
    return;
  }
  if (pwr >= 0) {
    analogWrite(negativePin, 0);
    analogWrite(positivePin, pwr);
    // Serial.print(pwr);
    // Serial.print(", ");
    // Serial.println(0);
  } else {
    analogWrite(positivePin, 0);
    analogWrite(negativePin, -pwr);
    // Serial.print(0);
    // Serial.print(", ");
    // Serial.println(-pwr);
  }
}
