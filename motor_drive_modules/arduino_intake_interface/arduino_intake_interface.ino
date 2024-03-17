#define positivePin 3
#define negativePin 5

volatile unsigned long pulseHighTime = 0;
volatile unsigned long pulseLowTime = 0;
volatile unsigned long lastRiseTime = 0;
volatile bool isHigh = false;
int dutyCycle = 0;
int maxDutyCycle = 1100;
int minDutyCycle = 242;
int power = 0;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT); // Using digital pin 2 for PWM input
  pinMode(positivePin, OUTPUT);
  pinMode(negativePin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), measurePulse, CHANGE);
}

void loop() {
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
  Serial.print(dutyCycle);
  Serial.print(" ");
  power = map(dutyCycle, minDutyCycle, maxDutyCycle, -255, 255);
  // Truncate power to [-1023, 1023]
  power = max(-255, min(power, 255));
  setPower(power);
}

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

void setPower(int pwr) {
  bool eat = 0;
  if (abs(pwr) < 140) {
    // This little power will not turn the motor.
    // Thus setting outputs to 0
    analogWrite(negativePin, 0);
    analogWrite(positivePin, 0);
    Serial.print(0);
    Serial.print(", ");
    Serial.println(0);
    return;
  }
  if (pwr >= 0) {
    analogWrite(negativePin, 0);
    analogWrite(positivePin, pwr);
    Serial.print(pwr);
    Serial.print(", ");
    Serial.println(0);
  } else {
    analogWrite(positivePin, 0);
    analogWrite(negativePin, -pwr);
    Serial.print(0);
    Serial.print(", ");
    Serial.println(-pwr);
  }
}
