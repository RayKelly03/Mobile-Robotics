#include <DRV8835MotorShield.h>

// Pin definitions
#define LED_PIN 39
#define M1PWM 37
#define M1Phase 38
#define M2PWM 39  
#define M2Phase 20

// Motor driver library call
DRV8835MotorShield motors(M1Phase, M1PWM, M2Phase, M2PWM);

// Variables
int minValues[5] = {0};
int maxValues[5] = {0};
int threshold[5] = {0};

// Pin assignments
int pins[] = {4, 5, 6, 7, 15}; // Define the pins to be calibrated
int numPins = 5; // Get the number of pins

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);

  // Assign initial values to min and max
  for (int i = 0; i < numPins; i++) {
    minValues[i] = analogRead(pins[i]);
    maxValues[i] = analogRead(pins[i]);
  }
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  calibrate();
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
/*
  while (true) {
    // Case for a sharp left turn or right turn
    if (analogRead(pins[1]) < threshold[1] && analogRead(pins[4]) > threshold[4]) {
      motors.setM1Speed(0);
      motors.setM2Speed(200);
    } else if (analogRead(pins[1]) > threshold[1] && analogRead(pins[4]) < threshold[4]) {
      motors.setM1Speed(200);
      motors.setM2Speed(0);
    }

    // Regular line follow using PID or otherwise
    else if (analogRead(pins[3]) < threshold[3]) {
      pid(); // To be implemented
    }
  }
}

*/

// To be done later
void pid() {
  // Placeholder for PID logic
}

// Spins the bot to calibrate thresholds for white and black surfaces
////using new millis fucntion to better time the loops
void calibrate() {
  unsigned long startTime = millis();  // Record the start time
  unsigned long calibrationDuration = 20000;  // Run calibration for 20 seconds

  motors.setSpeeds(150, -150);  // Spin the bot continuously during calibration

  while (millis() - startTime < calibrationDuration) {
    // Update min and max sensor values
    for (int j = 0; j < numPins; j++) {
      int value = analogRead(pins[j]);
      if (value < minValues[j]) {
        minValues[j] = value;
      }
      if (value > maxValues[j]) {
        maxValues[j] = value;
      }
    }
  }

  // Calculate thresholds
  for (int i = 0; i < numPins; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }

  // Print min, max, and threshold values for each sensor
  Serial.println("Calibration complete!");
  for (int i = 0; i < numPins; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" - Min: ");
    Serial.print(minValues[i]);
    Serial.print(", Max: ");
    Serial.print(maxValues[i]);
    Serial.print(", Threshold: ");
    Serial.println(threshold[i]);
  }

  motors.setSpeeds(0, 0);  // Stop the motors after calibration
}
