#define positivePin A0
#define negativePin A1

volatile unsigned long pulseHighTime = 0;
volatile unsigned long pulseLowTime = 0;
volatile unsigned long lastRiseTime = 0;
volatile bool isHigh = false;
int dutyCycle = 0;
int maxDutyCycle = 1100;
int minDutyCycle = 245;
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
  power = map(dutyCycle, minDutyCycle, maxDutyCycle, -1023, 1023);
  // Truncate power to [-1023, 1023]
  power = max(-1023, min(power, 1023));
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
