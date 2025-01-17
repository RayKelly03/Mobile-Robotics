
int AnalogValue[5] = {0,0,0,0,0};
int AnalogPin[5] = {4,5,6,7,15}; // keep 8 free for tone O/P music

int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 39;
int motor2Phase = 20;
int speed = 10 ;

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
  analogWrite(motor1PWM, speed);
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

void loop(){
  // put your main code here, to run repeatedly:
  int i;
  for (i=0;i<5;i++) {
    AnalogValue[i]=analogRead(AnalogPin[i]);

    Serial.print(AnalogValue[i]); // This prints the actual analog reading from the sensors
    Serial.print("\t"); //tab over on screen
    if(i==4) {
      Serial.println(""); //carriage return
      delay(600); // display new set of readings every 600mS
    }
  }
  while (AnalogValue[2] < 250) {
    Stop();
    Forward();
    AnalogValue[2]=analogRead(AnalogPin[2]);
   
  } 
  while (AnalogValue[0] < 250) {
    Stop();
    Left();
    AnalogValue[0]=analogRead(AnalogPin[0]);
    
  }

  while (AnalogValue[4] < 250) {
    Stop();
    Right();
    AnalogValue[4]=analogRead(AnalogPin[4]);
  }
}
