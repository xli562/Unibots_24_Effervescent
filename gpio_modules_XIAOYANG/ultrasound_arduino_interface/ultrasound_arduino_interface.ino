#include <NewPing.h>

// #define ULTRASONIC_TRIGGER_PIN  6  // Arduino Digital
// #define ULTRASONIC_ECHO_PIN     11  // Arduino Digital
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define NUM_ULTRASOUND 2

int trigPins[NUM_ULTRASOUND] = {6,7};  //Left_Bottom, Left_Top, Right, Front, Back
int echoPins[NUM_ULTRASOUND] = {11,12}; 

NewPing ultrasound_sensors[NUM_ULTRASOUND]{
  NewPing(trigPins[0], echoPins[0], MAX_DISTANCE),
  NewPing(trigPins[1], echoPins[1], MAX_DISTANCE)
};


void setup() {
  Serial.begin(115200); // Initialize serial communication.

//   for (int i = 0; i < NUM_ULTRASOUND; i++) {
//     ultrasound_sensors[i] = NewPing(trigPins[i], echoPins[i], MAX_DISTANCE);
//   } 
}

void loop() {
  for (int i = 0; i < NUM_ULTRASOUND; i++) {
    // Read ultrasonic sensor distance
    delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    unsigned int ultrasonic_distance = ultrasound_sensors[i].ping_cm(); // Send ping, get distance in centimeters and print result (0 = outside set distance range)
  
    // Send data over serial with identifiers
    Serial.print("U"); // Identifier for ultrasonic sensor
    Serial.print(i); // Add index of the sensor for identification
    Serial.print(":");
    Serial.print(ultrasonic_distance);
    Serial.print(",");
  }
  Serial.println("");
}
