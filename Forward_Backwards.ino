

int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 39;
int motor2Phase = 20;
// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  pinMode(motor1PWM,OUTPUT);
  pinMode(motor1Phase,OUTPUT);
  pinMode(motor2PWM,OUTPUT);
  pinMode(motor2Phase,OUTPUT);
}
// the loop routine runs over and over again continuously:
void loop() {
  digitalWrite(motor1Phase, HIGH); //forward
  digitalWrite(motor2Phase, HIGH); 
  analogWrite(motor1PWM, 100); // set speed of motor
  analogWrite(motor2PWM, 100); // set speed of motor
  Serial.println("Forward"); // Display motor direction
  delay(2000); //2 seconds
  digitalWrite(motor1Phase, LOW); //Backward
  digitalWrite(motor2Phase, LOW); //Backward
  analogWrite(motor1PWM, 100); // set speed of motor
  analogWrite(motor2PWM, 100); // set speed of motor
  Serial.println("Backward"); // Display motor direction
  delay(2000); //2 seconds
}
