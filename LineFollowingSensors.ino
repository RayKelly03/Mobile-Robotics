int AnalogValue[5] = {0,0,0,0,0};
int AnalogPin[5] = {4,5,6,7,15}; // keep 8 free for tone O/P music

int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 39;
int motor2Phase = 20;

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

void loop(){
  // put your main code here, to run repeatedly:
  int i;
  for (i=1;i<4;i++) {
    AnalogValue[i]=analogRead(AnalogPin[i]);

    Serial.print(AnalogValue[i]); // This prints the actual analog reading from the sensors
    Serial.print("\t"); //tab over on screen
    if(i==3) {
      Serial.println(""); //carriage return
      delay(600); // display new set of readings every 600mS
    }
  }
  while (AnalogValue[2] < 250) {
    Forward();
    AnalogValue[2]=analogRead(AnalogPin[2]); //middle Sensor
  }
  while (AnalogValue[2] > 250) {
    Stop(motor1PWM);
    Stop(motor2PWM);
    AnalogValue[2]=analogRead(AnalogPin[2]); //middle Sensor
  }
  while (AnalogValue[1] < 250) {
    Left();
    AnalogValue[1]=analogRead(AnalogPin[1]); //Left Sensor
  }
  while (AnalogValue[1] > 250) {
    Stop(motor2PWM);
    AnalogValue[1]=analogRead(AnalogPin[1]); //Left Sensor
  }

  while (AnalogValue[3] < 250) {
    Right();
    AnalogValue[3]=analogRead(AnalogPin[3]); //Right Sensor
  }
  while (AnalogValue[3] > 250) {
    Stop(motor1PWM);
    AnalogValue[1]=analogRead(AnalogPin[1]); //Left Sensor
  }
}
void Forward() {
  digitalWrite(motor1Phase, HIGH); //forward
  digitalWrite(motor2Phase, HIGH); 
  analogWrite(motor1PWM, 100); // set speed of motor
  analogWrite(motor2PWM, 100); // set speed of motor
  Serial.println("Forward"); // Display motor direction
}

void Right() {
  digitalWrite(motor1Phase, HIGH); 
  analogWrite(motor1PWM, 100); // set speed of motor
  Serial.println("Right"); // Display motor direction
}

void Left() {
  digitalWrite(motor2Phase, HIGH); 
  analogWrite(motor2PWM, 100); // set speed of motor
  Serial.println("Left"); // Display motor direction
}

void Stop(int MotorPwm) {
  analogWrite(MotorPwm, 0); 
  Serial.println("Stop");
}
