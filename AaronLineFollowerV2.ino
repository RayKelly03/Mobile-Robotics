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
  calibrate();
  delay(1000);

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

// To be done later
void pid() {
  // Placeholder for PID logic
}

// Spins the bot to calibrate thresholds for white and black surfaces
void calibrate() {
  // Run the motors and calibrate min and max values
  for (int i = 0; i < 3000; i++) {
    motors.setM1Speed(100);
    motors.setM2Speed(-100);

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

  // Calculate thresholds and print them
  for (int i = 0; i < numPins; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();

  // Stop the motors
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}
