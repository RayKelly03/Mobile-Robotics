#include <DRV8835MotorShield.h>

//pin definitions
#define LED_PIN 39
#define M1PWM 37
#define M1Phase 38
#define M2PWM 39  
#define M2Phase 20

//motor driver library call
DRV8835MotorShield motors(M1Phase, M1PWM, M2Phase, M2PWM);

//variables
int minValues[5]
int max0Values[5]
int threshold[5]

//pin assignments
int pins[] = {4, 5, 6, 7, 15}; // Define the pins to be calibrated
int numPins = 5; // Get the number of pins

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);


  //assign pin 
  for (int i = 0; i < numPins; i++){
    minValues[i] = analogRead(pins[i]);
    maxValues[i] = analogRead(pins[i]);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  calibrate();
  delay(1000);
  //maybe set up a button input after calibration to activate line following

  while(){

    //case for a sharp left turn or right turn
    if (analogRead(pins[1]) < threshold[1] && analogRead(pins[5]) > threshold[5] )
    {
      motors.setM1Speed(0)
      motors.setM2Speed(200)
    }
    else if (analogRead(pins[1]) > threshold[1] && analogRead(pins[5]) < threshold[5] )
    {
      motors.setM1Speed(200)
      motors.setM2Speed(0)
    }

    //regular line follow using PID or otherwise
    else if (analogRead(pins[3]) < threshold[3]){
      pid();// to be completed
    }


  }

}

//to be done later 
void pid(){

}



//spins the bot, to scan the area underneath to measure the white and black 
//so it can automatically select the most appropriate threshold value to 
//differentiate between black and white.
void calibrate() {

  // Initialize min and max values for the specified pins
  // moved to set up

  // Run the motors and calibrate min and max values
  for (int i = 0; i < 3000; i++){
    motors.setM1Speed(100)
    motors.setM2Speed(-100)
    
    for (int j = 0; j < numPins; j++){
      int value = analogRead(pins[j]);
      if (value < minValues[j])
      {
        minValues[j] = value;
      }
      if (value > maxValues[j])
      {
        maxValues[j] = value;
      }
    }
  }

  // Calculate thresholds and print them
  for (int i = 0; i < numPins; i++){
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();

  // Stop the motors
  motors.setM1Speed(0)
  motors.setM2Speed(0)
}