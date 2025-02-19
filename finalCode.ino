#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <MPU6050.h>
#include <limits.h>
#include <DRV8835MotorShield.h>

//#include <ESP32Servo.h>

#define SENSOR_PIN 16  
#define BUZZER_PIN 17  // Buzzer for alerts

#define ledPin1 21
#define ledPin2 47 

#define M1PWM 37
#define M1Phase 38
#define M2PWM 39  
#define M2Phase 20
#define BUFSIZE 512
#include <SPI.h>
#include <WiFi.h>
#include <WiFiServer.h>

// Motor driver library call
DRV8835MotorShield motors(M1Phase, M1PWM, M2Phase, M2PWM);

// Node A = Node 6, Node B = Node 7
#define I2C_SDA 13//OLED Pins 
#define I2C_SCL 14

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int obstacleThreshold = 2100;
int CLOSE_RANGE = 1700;
int MID_RANGE = 1500;
int FAR_RANGE = 1400;
bool obstacleFlag = false;

//Servo HeadServo;
const int servoPin = 9;

int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 39;
int motor2Phase = 20;

int threshold = 1000;
int yaw = 0;
int i = 0;
int j = 1;

int Inf = INT_MAX;
int prev = -1;
int next = -1;
int serverRoute[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
int routeSize ;
int analogValue[5] = {0, 0, 0, 0, 0};  // Store sensor values
int analogPin[5] = {4, 5, 6, 7, 15};    // Sensor pins

bool nodeFlag = false;

int Speed = 350; //standard speed
int turnSpeed = 248;
int LSP = 0; // speed variables for setting through PID
int RSP = 0;

// Pin assignments
int pins[] = {4, 5, 6, 7, 15}; // Define the pins to be calibrated
int numPins = 5; // Get the number of pins


//PID PROPERTIES
int P = 0;
int D = 0;
int I = 0; 
int previousError = 0;
int PIDval = 0;
int error = 0;
float Kp = 0;
float Ki = 0;
float Kd= 0;

//WIFI Details
char ssid[] = "iot";
char password[] = "pashalik20pedipulation";
String postBody;
String position;
String r;
String s;
String r1;
String s1;
int bk=0;
int cp; //current position
char server[] = "3.250.38.184";
int port = 8000;
WiFiClient client;

void connectToWiFi() {
  Serial.print("Connecting to network: ");  
  Serial.print(ssid);
  Serial.flush();
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    Serial.flush();
    delay(300);
  }
  Serial.println("Connected");
  Serial.flush();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool connect() {
  Serial.println("Connecting to server");
  if (!client.connect(server, port)) {
    Serial.println("error connecting to server");
    return false;
  }
  Serial.println("Connected to server");
  return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// read buffer size for HTTP response
String readResponse() {
  char buffer[BUFSIZE];
  memset(buffer, 0, BUFSIZE);
  client.readBytes(buffer, BUFSIZE);
  return String(buffer);
}

String getResponseBody(String& response) {
  int split = response.indexOf("\r\n\r\n");
  String body = response.substring(split+4);
  body.trim();
  return body;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateServer() {
  connect()     ;
  client.println("POST /api/arrived/lkim7619 HTTP/1.1");
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println("Connection: close");
  client.println();
  client.println(postBody);
  r = readResponse();
  s = getResponseBody(r);
  client.stop();
  position = s;
  postBody = "position=";
  postBody += position;
  Serial.print("Next Node: ") ;
  Serial.println(position);
}

void getRoute() {
  connect(); 
  client.println("GET /api/getRoute/lkim7619 HTTP/1.1");
  client.println("Host: 3.250.38.184");
  client.println("Connection: close");
  client.println();
  r1 = readResponse();
  s1 = getResponseBody(r1);
  client.stop();
  convertArray(s1) ;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Graph {
  private: 
    int adjMatrix[8][8]; // Initialises adjacency matrix
    int dir[8][8];
    int finalDir[8][8];
  public:
    Graph() {
      
        for (int i = 0; i < 8; ++i) {
            for (int j = 0; j < 8; ++j) {
                adjMatrix[i][j] = 0;
                dir[i][j] = -1;
                finalDir[i][j] = -1;
            }
        }
    }

    // Adds weighted edge to graph for connections between nodes
    void addEdge (unsigned int v, unsigned int w, int weight, int initialDir, int finalDire) {
      if (v < 8 && w < 8) { // Ensures node exists
        adjMatrix[v][w] = weight;
        dir[v][w] = initialDir;
        finalDir[v][w] = finalDire;
      }
      else {
            Serial.println("Error: Vertex out of bounds"); // If node doesn't exist
        }
    }

    int matrix(int prev, int next) {
      return adjMatrix[prev][next];
    }

    void removeEdge (unsigned int v, unsigned int w) {
      if (v < 8 && w < 8) { // Ensures node exists
        adjMatrix[v][w] = 0;
        adjMatrix[w][v] = 0; // Removes the link between the two nodes so it won't be considered in route planning
      }
      else {
            Serial.println("Error: Vertex out of bounds"); // If node doesn't exist
        }
    }

    // Prints Adjacency Matrix - only for testing purposes
    void displayMatrix() {
        for (int i = 0; i < 8; ++i) {
            for (int j = 0; j < 8; ++j) {
                Serial.print(adjMatrix[i][j]);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

    int returnInitialDir (int prev, int next) {
      return dir[prev][next];
    }

    int returnFinalDir (int prev, int next) {
      return finalDir[prev][next];
    }

    // Finds closest unvisited node to the graph
    int minDistance(int dist[], bool sptSet[]) {
      int min = Inf; 
      int min_index = -1; 
  
      for (int v = 0; v < 8; v++) {
          if (!sptSet[v] && dist[v] <= min) { //If node is unvisited and closer than the other unvisited nodes
            min = dist[v]; //updates the new min variable to the distance to the unvisited node
            min_index = v;
          }
      }
      return min_index; // Returns the index of the closest unvisited node
    }


    void dijkstra(int src, int dest, int route[]) {
        int dist[8];      // Shortest total distance from src to other nodes
        bool sptSet[8];   // Array of processed nodes
        int parent[8];    // Array of parent of each node (acts as a lookup table)

        // Initialize distances to Inf and sptSet to false
        for (int i = 0; i < 8; i++) {
            dist[i] = Inf;
            sptSet[i] = false;
            parent[i] = -1;
        }

        dist[src] = 0; // Distance from source to source set to 0

        // Finds the shortest path for all vertices
        for (int count = 0; count < 8 - 1; count++) {
            int u = minDistance(dist, sptSet); // Finds closest unvisited node
            sptSet[u] = true; // Mark the vertex as processed

            // Update the distance value of adjacent vertices to see if shorter route has been found
            for (int v = 0; v < 8; v++) {
                // Checks conditions for if a shorter route to a node from src has been found
                if (!sptSet[v] && adjMatrix[u][v] && dist[u] != Inf && dist[u] + adjMatrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + adjMatrix[u][v];
                    parent[v] = u; // Update the parent
                }
            }
        }

        int index = 0;
        printPath(parent, dest, route, index);
    }

    void printPath(int parent[], int j, int route[], int &index) {
        if (parent[j] == -1) {
            route[index++] = j;
            Serial.print(j);
            return;
        }
        printPath(parent, parent[j], route, index);
        route[index++] = j;
        Serial.print(" -> ");
        Serial.print(j);
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Graph g; // Creates Graph

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void convertArray(String data) {
  routeSize = 0;  
  int startIndex = 0, endIndex;

  while ((endIndex = data.indexOf(',', startIndex)) != -1 && routeSize < 10) {
    serverRoute[routeSize++] = data.substring(startIndex, endIndex).toInt();
    startIndex = endIndex + 1;
  }

  if (startIndex < data.length() && routeSize < 10) {
   serverRoute[routeSize++] = data.substring(startIndex).toInt();
  }

  for (int i = routeSize; i < 10; i++) {
    serverRoute[i] = -1;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pid(){
  int error = (analogRead(pins[1])-analogRead(pins[3]));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDval = (Kp * P) + (Ki * I) + (Kd * D) + 20; 
  // +60 is an offset value to bring the final PIDval to 0 when on the line
  previousError = error;

  LSP = Speed - PIDval;
  RSP = Speed + PIDval;

  if (LSP > Speed) {
    LSP = Speed;
  }
  if (LSP < -0){
    LSP = -0;
  }
  if (RSP > Speed){
    RSP = Speed;
  }
  if (RSP < -0){
    RSP = -0;
  }
  motors.setSpeeds(RSP, LSP); // switch to deviate toward/away from line
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void followLine() {
    motors.setSpeeds(Speed, Speed);
    delay(200);

    while(true) {
      if (analogRead(pins[0]) < threshold && analogRead(pins[4]) > threshold) {
        motors.setSpeeds(50, turnSpeed);
      } 
      //Right turn
      else if (analogRead(pins[0]) > threshold && analogRead(pins[4]) < threshold) {
        motors.setSpeeds(turnSpeed, 50);
      }
      //NODE Code
      else if (analogRead(pins[1]) < threshold && analogRead(pins[2]) < threshold && analogRead(pins[3]) < threshold ) {
        if (nodeFlag) {
          motors.setSpeeds(0, 0);
          digitalWrite(ledPin1, HIGH);
          digitalWrite(ledPin2, HIGH);
          nodeFlag = false;
        }
        else {
          delay(50);
        }
        break;
      }

      else if (analogRead(pins[2]) < threshold || analogRead(pins[1]) < threshold || analogRead(pins[3]) < threshold) {
        Kd = 0; 
        Ki = 0;
        Kp = 0.1; // Speed variable / (Max sensor Reading / 2)
        pid(); // To be implemented
      }

      if (analogRead(SENSOR_PIN) > 3300) {
        Serial.println(analogRead(SENSOR_PIN));
        motors.setSpeeds(0, 0);
        digitalWrite(ledPin1, HIGH);
        digitalWrite(ledPin2, HIGH);
        obstacleFlag = true;
        break;
      }

      for (int i = 0; i < 5; i++) {
          analogValue[i] = analogRead(analogPin[i]);
      } 
    }
  }

  void followSensorLine() {
    while(true) {
      if (analogRead(pins[0]) < threshold && analogRead(pins[4]) > threshold) {
        motors.setSpeeds(50, turnSpeed);
      }

      //Right turn
      else if (analogRead(pins[0]) > threshold && analogRead(pins[4]) < threshold) {
        motors.setSpeeds(turnSpeed, 50);
      }

      else if (analogRead(pins[1]) < threshold && analogRead(pins[3]) > threshold) {
        motors.setSpeeds(100, -100);
      }

      else if (analogRead(pins[3]) < threshold && analogRead(pins[1]) > threshold) {
        motors.setSpeeds(-100, 100);
      }

      else if (analogRead(pins[2]) < threshold && analogRead(pins[1]) < threshold) {
        motors.setSpeeds(100, -100);
      }
      
      else if (analogRead(pins[3]) < threshold && analogRead(pins[2]) < threshold) {
        motors.setSpeeds(-100, 100);
      }
      
      // Regular line follow using PID or otherwise
      else if (analogRead(pins[2]) < threshold && analogRead(pins[3]) > threshold && analogRead(pins[1]) > threshold) {
        motors.setSpeeds(0, 0);
      }

      if (analogRead(pins[0]) > threshold && analogRead(pins[4]) > threshold && analogRead(pins[1]) > threshold && analogRead(pins[3]) > threshold && analogRead(pins[2]) > threshold) {
        break;
      }

      for (int i = 0; i < 5; i++) {
        analogValue[i] = analogRead(analogPin[i]);
      } 
      delay(1);
    }
    Speed = 300;
  }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setTimeAng(int targetAngle, int prev, int next) {
    if (targetAngle == yaw) {
        return;  // Exit if already at the target angle
    }

    motors.setSpeeds(100, 100); 
    delay(700);
    motors.setSpeeds(0, 0);
    delay(200);

    int angleDifference = targetAngle - yaw;  
   while (angleDifference > 180) {
        angleDifference -= 360;
    }
    while (angleDifference < -180) {
        angleDifference += 360;
    }

    int turnDirection = (angleDifference > 0) ? 1 : -1; // 1 = Right, -1 = Left

    int nodes = (abs(angleDifference)/90);
    int turnTime = nodes * 800;

    // Start turning
    if (nodes != 0) { // double check calculation
        if (turnDirection == 1) {
          motors.setSpeeds(200, -200);
          delay(turnTime);
        } 
        else  {
          motors.setSpeeds(-200, 200);
          delay(turnTime);
        }
    }

    motors.setSpeeds(0, 0);
    delay(500); // Small delay before following the line
}

void path(int prev, int next) {
    int angle = g.returnInitialDir(prev, next);
    setTimeAng(angle, prev, next);

    if (next == 5) {
      followSensorLine();
      checkParkingSensor();
    }
    else {
      followLine();
    }
    yaw = g.returnFinalDir(prev, next);
}

void serverPath(int prev, int next) {
  int route[10];
  int a = 0;
  int b = 1;

  for (int k = 0; k < 10; k++) { 
    route[k] = -1;
  }

  g.dijkstra(prev, next, route);
  
  while (route[b] != -1) {
    
    if (route[b] == next) {
      nodeFlag = true;
    }
    path(route[a], route[b]);
    
    if (obstacleFlag == true) {
      g.removeEdge(route[a], route[b]);
      motors.setSpeeds(200, -200);
      nodeFlag = true;
      delay(1600);
      followLine();
      nodeFlag = false;

      yaw = g.returnFinalDir(route[b], route[a]);

      int redirectRoute[10]; 
      for (int k = 0; k < 10; k++) {
        redirectRoute[k] = -1;
      }
      g.dijkstra(route[a], next, redirectRoute);

      for (int i = 0; i < 10; i++) {
        route[i] = redirectRoute[i];
      }
      a = -1;
      b = 0;

      obstacleFlag = false;
    }
    
    a++;
    b++;
  }
  updateServer();
  motors.setSpeeds(0, 0); 
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
  nodeFlag = false;
  delay(200);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void checkParkingSensor() {  
  Serial.println("Entering wall detection mode...");
  motors.setSpeeds(282, 300);  

  while (true) {
    int sensorValue = analogRead(SENSOR_PIN);  
    Serial.print("Sensor Value: ");
    Serial.println(sensorValue);

    beepBuzzer(sensorValue);

    if (sensorValue > obstacleThreshold) {
      Serial.println("Wall detected! Stopping permanently...");
      motors.setSpeeds(0, 0);  // Stop the robot
      digitalWrite(ledPin1, HIGH);
      digitalWrite(ledPin2, HIGH);
      break;
    }
    
    delay(100);  
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void beepBuzzer(int sensorValue) {
  static unsigned long lastToggleTime = 0;
  static bool buzzerOn = false;
  unsigned long currentTime = millis();
  
  // Define on/off durations based on sensor reading 
  unsigned long onDuration = 0;
  unsigned long offDuration = 0;
  
  if (sensorValue > 2000) {       // Too close: almost continuous beep
    onDuration = 200;             // Buzzer on for 200ms
    offDuration = 50;             // Off for 50ms
  } 
  else if (sensorValue > 500) {  // Very close: rapid beeping
    onDuration = 100;             // On for 100ms
    offDuration = 100;  
  }
  else if (sensorValue < 500) {  // Close: moderate beeping
    onDuration = 70;              // On for 70ms
    offDuration = 180; 
  }
  
  else {
    // No obstacle
    digitalWrite(BUZZER_PIN, LOW);
  //  digitalWrite(LED_PIN, LOW);
    buzzerOn = false;
    lastToggleTime = currentTime;
    return;
  }
  
  // Toggle buzzer state based on elapsed time 
  if (buzzerOn && (currentTime - lastToggleTime >= onDuration)) {
    digitalWrite(BUZZER_PIN, LOW);
   // digitalWrite(LED_PIN, LOW);
    buzzerOn = false;
    lastToggleTime = currentTime;
  }
  else if (!buzzerOn && (currentTime - lastToggleTime >= offDuration)) {
    digitalWrite(BUZZER_PIN, HIGH);
   // digitalWrite(LED_PIN, LOW);
    buzzerOn = true;
    lastToggleTime = currentTime;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(115200);
    delay(1000);
    motors.setSpeeds(0, 0);
    
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);

    pinMode(motor1PWM,OUTPUT);
    pinMode(motor1Phase,OUTPUT);
    pinMode(motor2PWM,OUTPUT);
    pinMode(motor2Phase,OUTPUT);

    connectToWiFi();
    getRoute();

    //HeadServo.attach(servoPin);
    //HeadServo.write(90);
    delay(500);  

    g.addEdge(0, 6, 78, 0, 270);
    g.addEdge(6, 0, 78, 90, 180);
    g.addEdge(0, 4, 80, 180, 180);
    g.addEdge(4, 0, 80, 0, 0);
    g.addEdge(1, 6, 60, 0, 0);
    g.addEdge(6, 1, 60, 180, 180);
    g.addEdge(1, 7, 23, 180, 180);
    g.addEdge(7, 1, 23, 0, 0);
    g.addEdge(2, 6, 78, 0, 90);
    g.addEdge(6, 2, 78, 270, 180);
    g.addEdge(2, 3, 80, 180, 180);
    g.addEdge(3, 2, 80, 0, 0);
    g.addEdge(3, 7, 150, 180, 90);
    g.addEdge(7, 3, 150, 270, 0);
    g.addEdge(4, 7, 150, 180, 270);
    g.addEdge(7, 4, 150, 90, 0);
    g.addEdge(7, 5, 1, 180, 180);   // Creates Adjacency Matrix in format : addEdge(Node A, Node B, distance between)

    for (int i = 0; i < 5; i++) {
      pinMode(analogPin[i], INPUT);  // Sensor pins setup
    }

    postBody="position=";
    position="0";
    postBody += position;
    nodeFlag = true;
    followLine();
    nodeFlag = false;
    updateServer();
}


void loop() {

  while (serverRoute[j] != -1) {
    serverPath(serverRoute[i], serverRoute[j]);
    i++;
    j++;
  }

  while(true) {
    delay(1000);
  }

}
