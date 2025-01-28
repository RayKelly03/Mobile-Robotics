

int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 39;
int motor2Phase = 20;

// the setup routine runs once when you press reset:

void Forward() {
  digitalWrite(motor1Phase, LOW); //forward
  digitalWrite(motor2Phase, LOW); 
  analogWrite(motor1PWM, 100); // set speed of motor
  analogWrite(motor2PWM, 100); // set speed of motor
  Serial.println("Forward"); // Display motor direction
}

void Backward() {
  digitalWrite(motor1Phase, HIGH); //forward
  digitalWrite(motor2Phase, HIGH); 
  analogWrite(motor1PWM, 100); // set speed of motor
  analogWrite(motor2PWM, 100); // set speed of motor
  Serial.println("Backward"); // Display motor direction
}

void Right() {
  digitalWrite(motor1Phase, LOW); 
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, 100); // set speed of motor
  Serial.println("Right"); // Display motor direction
}

void Left() {
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, LOW); 
  analogWrite(motor2PWM, 100); // set speed of motor
  Serial.println("Left"); // Display motor direction
}

void setup() {
  Serial.begin(9600);
  pinMode(motor1PWM,OUTPUT);
  pinMode(motor1Phase,OUTPUT);
  pinMode(motor2PWM,OUTPUT);
  pinMode(motor2Phase,OUTPUT);
  analogWrite(motor1PWM, 100); // set speed of motor
  analogWrite(motor2PWM, 100); // set speed of motor
}

// the loop routine runs over and over again continuously:
void loop() {
  Forward();
  delay(2000);
  Right();
  delay(2000); //2 seconds
  Left();
  delay(2000);
  Backward();  
  delay(2000); //2 seconds
}
