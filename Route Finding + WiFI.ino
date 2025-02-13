#include <Wire.h>
#include <MPU6050.h>
#include <limits.h>
#include <DRV8835MotorShield.h>
#define BUFSIZE 512
#include <SPI.h>
#include <WiFi.h>
#include <WiFiServer.h>

#define SENSOR_PIN 16  
#define BUZZER_PIN 17  // Buzzer for alerts

#define M1PWM 37
#define M1Phase 38
#define M2PWM 39  
#define M2Phase 20

// Motor driver library call
DRV8835MotorShield motors(M1Phase, M1PWM, M2Phase, M2PWM);



// Node A = Node 6, Node B = Node 7


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
int routeSize = 0;  
int analogValue[5] = {0, 0, 0, 0, 0};  // Store sensor values
int analogPin[5] = {4, 5, 6, 7, 15};    // Sensor pins


int Speed = 330; //standard speed
int turnSpeed = 262.5;
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
char password[] = "weirdish88prozymite";
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
  //Serial.print("Obtaining IP address");
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
  Serial.print("Next Node: ") ;
  Serial.println(position);
  c
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
                dir[i][j] = 0;
                finalDir[i][i] = 0;
            }
        }
    }

    // Adds weighted edge to graph for connections between nodes
    void addEdge (unsigned int v, unsigned int w, int weight, int initialDir, int finalDire) {
      if (v < 8 && w < 8) { // Ensures node exists
        adjMatrix[w][v] = weight;
        dir[w][v] = initialDir;
        finalDir[w][v] = finalDire;
      }
      else {
            Serial.println("Error: Vertex out of bounds"); // If node doesn't exist
        }
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

    // Print the distance and paths
    /*
    void printSolution(int dist[], int parent[], int src, int dest) {
        Serial.println("Vertex\tDistance\tPath");
        Serial.print(dest);
        Serial.print("\t");
        Serial.print(dist[dest]);
        Serial.print("\t\t");
        int index = 0;
        printPath(parent, dest, route, index);
        Serial.println();
    }
    */


    // Prints the shortest path - uses parent array
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

    while(true) {
      if (analogRead(pins[0]) < threshold && analogRead(pins[4]) > threshold) {
      motors.setSpeeds(50, turnSpeed);
    } 
    //Right turn
    else if (analogRead(pins[0]) > threshold && analogRead(pins[4]) < threshold) {
      motors.setSpeeds(turnSpeed, 50);
    }
    //NODE Code
    else if (analogRead(pins[0]) < threshold && analogRead(pins[4]) < threshold && analogRead(pins[1]) < threshold && analogRead(pins[3]) < threshold && analogRead(pins[2]) < threshold)  {
      Serial.print("Node ");
      //while(analogRead(pins[0]) < threshold && analogRead(pins[4]) < threshold && analogRead(pins[1]) < threshold && analogRead(pins[3]) < threshold){

      //}
      for (int i; i < 1000; i++ ) {
      }
      motors.setSpeeds(0, 0); //stop
      delay(200);
      break;
    }
    // Regular line follow using PID or otherwise
    else if (analogRead(pins[2]) < threshold || analogRead(pins[1]) < threshold || analogRead(pins[3]) < threshold) {
      //Kp = 0.0006 * (1000 - analogRead(pins[2]));
      Kd = 0; 
      Ki = 0;
      Kp = 0.1; // Speed variable / (Max sensor Reading / 2)
      pid(); // To be implemented
    }

    for (int i = 0; i < 5; i++) {
        analogValue[i] = analogRead(analogPin[i]);
        //Serial.print(analogValue[i]);
        //Serial.print("\t");
      } 
      //Serial.println("");
  }
  }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setGyroAng(int targetAngle, int prev, int next) {

    if (targetAngle == yaw) {
        return;  // Exit if already at the target angle
    }


    int angleDifference = targetAngle - yaw;  

    if (angleDifference > 180) {
        angleDifference -= 360;
    } 
    else if (angleDifference < -180) {
        angleDifference += 360;
    }


    int turnDirection = (angleDifference > 0) ? 1 : -1; // 1 = Right, -1 = Left
    int nodeCount = 0;
    int nodes = 0;
    
    if (abs(angleDifference) == 180) {
      nodes = 2;
    }
    else {
      nodes = abs(angleDifference/90) - 1;
    }


    while (analogRead(analogPin[2]) < threshold) {
    if (turnDirection == 1) {
          motors.setSpeeds(-200, 200);
    }
    else {
        motors.setSpeeds(200, -200);         
    }
  }

      Serial.print("Node Count before while loop = ");
      Serial.println(nodeCount);
      Serial.print("Nodes= ");
      Serial.println(nodes);
      
    // Start turning
    while(nodeCount <= nodes) {
        if (turnDirection == 1) {
          motors.setSpeeds(-200, 200);
        } 
        else {
          motors.setSpeeds(200, -200);
        }
      

      while (analogRead(analogPin[2]) <= 1000) {
        if (turnDirection == 1) {
          motors.setSpeeds(-200, 200);
        } 
        else {
          motors.setSpeeds(200, -200);
        }
        delay(300);  // Small delay to avoid false detections
        nodeCount++;
      }
    }

     // Wait for calculated turn time

    // Continue adjusting until a sensor detects a line


    while (analogRead(analogPin[2]) >= 1000 ) {
        // Keep turning
        if (turnDirection == 1) {
          motors.setSpeeds(200, -200);
        
        } else {
          motors.setSpeeds(-200, 200);
        }
    }

    //yaw = (targetAngle + 360) % 360;

    // Stop motors once a sensor detects a line
    motors.setM1Speed(0);
    motors.setM2Speed(0);
    delay(100); // Small pause

    // Adjust yaw estimation (since there's no gyro, update manually)
    yaw = g.returnFinalDir(next, prev);
    

    Serial.println("Yaw Target Achieved");

    delay(500); // Small delay before following the line
}


void path(int prev, int next) {
    int angle = g.returnInitialDir(next, prev);
    setGyroAng(angle, prev, next);
    if (next == 7) {
      checkParkingSensor();
    }
    else {
      followLine();
    }
}

void serverPath(int prev, int next) {
  int route[10];
  int a = 0;
  int b = 1;
  for(int k = 0; k < 10; k++) {
    route[k] = -1;
  }
  g.dijkstra(prev, next, route);
  while(route[b] != -1) {
    path(route[a], route[b]);
    a++;
    b++;
  }
  updateServer();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Function to check parking sensor and activate buzzer accordingly
void checkParkingSensor() {

  int OBSTACLE_THRESHOLD = 3200;  // 3500 for spare, normal is 1500 
  int CLOSE_RANGE = 3200;
  int MID_RANGE = 3000;
  int FAR_RANGE = 2900;

  int sensorValue = analogRead(SENSOR_PIN);  // Read sensor value
  int beepDelay = 0; // Time between beeps

  motors.setSpeeds(300, 300);

  // Adjust beep frequency based on distance
  while(true) {
    if (sensorValue > OBSTACLE_THRESHOLD) {
      Serial.println("Too Close!");
      beepDelay = 100;  // Fastest beep (closest range)
    } 
    else if (sensorValue > CLOSE_RANGE) {
      Serial.println("Very Close!");
      beepDelay = 250;
      motors.setSpeeds(0, 0);
      break;
    }
    else if (sensorValue > MID_RANGE) {
      Serial.println("Close!");
      beepDelay = 500;
    }
    else if (sensorValue > FAR_RANGE) {
      Serial.println("Approaching..");
      beepDelay = 1000; // Slowest beep (farthest range)
    } 
    else {
      Serial.println("No obstacle detected.");
      beepDelay = 0; // No beeping if no obstacle
    }

    if (beepDelay > 0) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(beepDelay - 100);  // Adjust off time based on beep delay
    } 
    
    else {
    delay(500);  // Default delay when no obstacle
    }
  }
}
  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(115200);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    motors.setSpeeds(0, 0);
    delay(1000);

    pinMode(motor1PWM,OUTPUT);
    pinMode(motor1Phase,OUTPUT);
    pinMode(motor2PWM,OUTPUT);
    pinMode(motor2Phase,OUTPUT);

    
    getRoute();
    
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
     

    Serial.print("Adjacency Matrix:");
    g.displayMatrix();
  

    for (int i = 0; i < 5; i++) {
    pinMode(analogPin[i], INPUT);  // Sensor pins setup
  }
    
}

void loop() {
  while (serverRoute[j] != -1) {
    serverPath(serverRoute[i], serverRoute[j]);
    i++;
    j++;
  }

}
