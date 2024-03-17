// Define the number of sensors
#define NUM_SENSORS 5

// Define the trig and echo pins for each ultrasonic sensor
int trigPins[NUM_SENSORS] = {2, 3, 4, 5, 6}; // Example pin assignments for trig pins
int echoPins[NUM_SENSORS] = {7, 8, 9, 10, 11}; // Example pin assignments for echo pins

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate

  // Initialize all trigPins as output and echoPins as input
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    long duration, distance;
    
    // Clear the trigPin
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);

    // Set the trigPin high for 10 microseconds
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    // Read the echoPin and calculate the distance
    duration = pulseIn(echoPins[i], HIGH);
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and return)

    // Print the distance for the current sensor
    Serial.print("Sensor ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  // Add a small delay between readings for stability
  delay(500);
}
