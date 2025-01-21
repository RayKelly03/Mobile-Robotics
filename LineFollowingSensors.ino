
int AnalogValue[5] = {0,0,0,0,0};
int AnalogPin[5] = {4,5,6,7,15}; // keep 8 free for tone O/P music

int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 39;
int motor2Phase = 20;
int speed = 100;

void Forward() {
  digitalWrite(motor1Phase, LOW); //forward
  digitalWrite(motor2Phase, LOW); 
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed); // set speed of motor
}

void Backward() {
  digitalWrite(motor1Phase, HIGH); //forward
  digitalWrite(motor2Phase, HIGH); 
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed); // set speed of motor
}

void Right() {
  digitalWrite(motor1Phase, LOW); 
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, speed);
  analogWrite(motor2PWM, speed);
}

void Left() {
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, (speed*0.75));
  analogWrite(motor2PWM, speed); // set speed of motor
}

void Stop() {
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0); 
}


void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(motor1PWM,OUTPUT);
    pinMode(motor1Phase,OUTPUT);
    pinMode(motor2PWM,OUTPUT);
    pinMode(motor2Phase,OUTPUT);
    for (int i=0;i<5;i++) {
      pinMode(AnalogPin[i] , INPUT) ;
    }
} 

void loop() {
  // Update all sensor readings
  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    Serial.print(AnalogValue[i]); // Print the sensor value
    Serial.print("\t"); // Tab spacing
    if (i == 4) {
      Serial.println(""); // New line after all readings
      delay(100); // Small delay for readability
    }
  }

  // Movement decision based on sensor readings
  if (AnalogValue[2] < 250) { 
    Forward();
    delay(50);
    Serial.println("Moving Forward (Middle)");
  } 
  else if (AnalogValue[1] < 250) {
    Left();
    delay(50);
    Serial.println("Turning Left");
  } 
  else if (AnalogValue[3] < 250) {
    Right();
    delay(50);
    Serial.println("Turning Right");
  } 
  else if(AnalogValue[4] < 250) {
    Right();
    delay(50);
  }
  else if(AnalogValue[0] < 250) {
    Left();
    delay(50);
  }
  else {
    Stop();
    Serial.println("Stopped");
  }
}
