#include <Wire.h>
#include <MPU6050.h>
#include <limits.h>
#include <DRV8835MotorShield.h>

#define M1PWM 37
#define M1Phase 38
#define M2PWM 39  
#define M2Phase 20

// Motor driver library call
DRV8835MotorShield motors(M1Phase, M1PWM, M2Phase, M2PWM);

// Node A = Node 6, Node B = Node 7

MPU6050 mpu;

unsigned long timer = 0;
float timeStep = 0.1;

int yaw = 0;
int threshold = 1000;

int Inf = INT_MAX;
int prev = -1;
int next = -1;
int route[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

int analogValue[5] = {0, 0, 0, 0, 0};  // Store sensor values
int analogPin[5] = {4, 5, 6, 7, 15};    // Sensor pins


int minValues[5] = {0};
int maxValues[5] = {0};

int Speed = 350; //standard speed
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Graph {
  private: 
    int adjMatrix[8][8]; // Initialises adjacency matrix
  public:
    Graph() {
      
        for (int i = 0; i < 8; ++i) {
            for (int j = 0; j < 8; ++j) {
                adjMatrix[i][j] = 0;
            }
        }
    }

    // Adds weighted edge to graph for connections between nodes
    void addEdge (unsigned int v, unsigned int w, int weight) {
      if (v < 8 && w < 8) { // Ensures node exists
        adjMatrix[v][w] = weight;
        adjMatrix[w][v] = weight; // Sets both locations for a route in the adjacency matrix to the distance between nodes 
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


    void dijkstra(int src, int dest) {
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

        // Print the results
        printSolution(dist, parent, src, dest);
    }

    // Print the distance and paths
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
/*
void calibrate(int yaw) {
  unsigned long startTime = millis();  // Record the start time
  unsigned long calibrationDuration = 5000;  // Run calibration for 3000 ms
  while (abs(yaw) < 360) {
    motors.setSpeeds(200, -200);  // Spin the bot continuously during calibration
  }

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

*/
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

  // Debugging output
  /*
  Serial.print("Error: "); Serial.print(error);
  Serial.print(", P: "); Serial.print(P);
  Serial.print(", Kp: "); Serial.print(Kp);
  Serial.print(", D: "); Serial.print(D);
  Serial.print(", Kd: "); Serial.print(Kd);
  Serial.print(", I: "); Serial.print(I);
  Serial.print(", LSP: "); Serial.print(LSP);
  Serial.print(", RSP: "); Serial.println(RSP);
  Serial.print(", PIDval: "); Serial.println(PIDval);
  */
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  /*
  while(!(analogRead(pins[0]) < 1000 && analogRead(pins[4] < 1000))){
    Serial.print(analogValue[0]);

    if (analogValue[0] > threshold && analogValue[4] < threshold) {
      Serial.println("Right");
      motors.setM1Speed(turnSpeed);
      motors.setM2Speed(50);
    }
    else if (analogValue[1] < threshold) {  //&& analogValue[4] > threshold
      Serial.println("Left");
      motors.setM1Speed(50);
      motors.setM2Speed(turnSpeed);
    } 

   // Regular line follow using PID or otherwise
    else if (analogValue[2] < threshold) {
      //Kp = 0.0006 * (1000 - analogRead(pins[2]));
      Kd = 0.025; 
      Ki = 0.0001;
      Kp = 0.05; // Speed variable / (Max sensor Reading / 2)
      pid(); // To be implemented
    }

    for (int i = 0; i < 5; i++) {
      analogValue[i] = analogRead(analogPin[i]);
      //Serial.print(analogValue[i]);
      //Serial.print("\t");
    }
    Serial.println("");
  
  }
    
  */

void followLine() {
  while(!(analogRead(pins[0]) < 1000 && analogRead(pins[4] < 1000))) {

    if (analogRead(pins[1]) < threshold && analogRead(pins[4]) > threshold) {
      motors.setM1Speed(50);
      motors.setM2Speed(turnSpeed);
    } 
    //Right turn
    else if (analogRead(pins[0]) > threshold && analogRead(pins[4]) < threshold) {
      motors.setM1Speed(turnSpeed);
      motors.setM2Speed(50);
    }
    //NODE Code
    else if (analogRead(pins[0]) < threshold && analogRead(pins[4]) < threshold) {
      Serial.print("Node ");
      motors.setSpeeds(400, 400); // incase speed value is changed
      motors.setSpeeds(0, 0); //stop
    }
    // Regular line follow using PID or otherwise
    else if (analogRead(pins[2]) < threshold) {
      //Kp = 0.0006 * (1000 - analogRead(pins[2]));
      Kd = 0.025; 
      Ki = 0.0001;
      Kp = 0.05; // Speed variable / (Max sensor Reading / 2)
      pid(); // To be implemented
    }

    delay(abs((timeStep * 1000) - (millis() - timer)));
    timer = millis();
    Vector normGyro = mpu.readNormalizeGyro();
    yaw = yaw + normGyro.ZAxis * timeStep;
  }
}

/*
void stop() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void path(int prev, int next) {
    
  if(next == 6 && prev == 0){
    setGyroAng(0);
    followLine();
  }

  if(next == 0 && prev == 6){
    setGyroAng(90); // positive angle for right turn, negative angle for left turn?
    followLine();
  }

  else if(next == 1 && prev == 6){
    setGyroAng(180);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 6 && prev == 1){
    setGyroAng(180);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 1 && prev == 7){
    setGyroAng(0);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 7 && prev == 1){
    setGyroAng(0);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 0 && prev == 4){
    setGyroAng(0);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 4 && prev == 0){
    setGyroAng(180);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 2 && prev == 6){
    setGyroAng(-90);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 6 && prev == 2){
    setGyroAng(0);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 3 && prev == 2){
    setGyroAng(0);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 2 && prev == 3){
    setGyroAng(180);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 3 && prev == 7){
    setGyroAng(180);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 7 && prev == 3){
    setGyroAng(-90);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 7 && prev == 4){
    setGyroAng(90);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 4 && prev == 7){
    setGyroAng(180);
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }

  else if(next == 5 && prev == 7){
    setGyroAng(180);
    
    // Insert Parking Code Here
    while (analogValue[0] > 300 && analogValue[1] > 300 && analogValue[2] > 300 && analogValue[3] > 300 && analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void removeEdgeRedirect(int gyroAngle, int prev,int next, Graph g) {
  setGyroAngle(gyroAngle + 180);
  while(analogValue[0], analogValue[1], analogValue[2], analogValue[3], analogValue[4] > 300) {
    followLine();
  }
  g.removeEdge(prev, next);
  g.dijkstra(prev);
  //path(prev, next);
  
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setGyroAng(int targetAngle) {
  /*
  Serial.print("targetAngle = ");
  Serial.print(targetAngle);
  Serial.print("    Yaw = ");
  Serial.println(yaw);
  */
  // Turn towards target angle
  while (abs(yaw - targetAngle) > 5) {  // Ensures the loop exits when within the threshold  
    
    if (yaw > targetAngle) {
      motors.setM1Speed(100);
      motors.setM2Speed(-100);
    } else {
      motors.setM1Speed(-100);
      motors.setM2Speed(100);
    }

    Serial.print(" Yaw = ");
    Serial.println(yaw);

    timer = millis();
    Vector normGyro = mpu.readNormalizeGyro();
    yaw = yaw + normGyro.ZAxis * timeStep;
    
    Serial.print(" Yaw = ");
    Serial.println(yaw);

    delay(abs((timeStep * 1000) - (millis() - timer)));
  }
  
  // Continue turning until sensor condition is met
  while (analogValue[2] > 1000) {
    if (yaw > targetAngle) {
      motors.setM1Speed(100);
      motors.setM2Speed(-100);
    } else {
      motors.setM1Speed(-100);
      motors.setM2Speed(100);
    }
    
    Serial.print(" Waiting for sensor... Yaw = ");
    Serial.println(yaw);
  }
  
  // Stop motors after reaching the desired angle AND sensor condition is met
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  Serial.println("Yaw Target Achieved, Stopping");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(115200);

    motors.setM1Speed(0);
    motors.setM2Speed(0);
    delay(1000);


    Graph g; // Creates Graph
    g.addEdge(0, 6, 78);
    g.addEdge(0, 4, 80);
    g.addEdge(1, 6, 60);
    g.addEdge(1, 7, 23);
    g.addEdge(2, 6, 78);
    g.addEdge(2, 3, 80);
    g.addEdge(3, 7, 150);
    g.addEdge(4, 7, 150);
    g.addEdge(7, 5, 1);   // Creates Adjacency Matrix in format : addEdge(Node A, Node B, distance between)

    Serial.print("Adjacency Matrix:");
    g.displayMatrix();
    g.dijkstra(0, 1);

    for (int i = 0; i < 5; i++) {
    pinMode(analogPin[i], INPUT);  // Sensor pins setup
  }

    // Initialize I2C with custom pins
    Wire.begin(, 48); // SDA = GPIO45, SCL = GPIO48

    // Initialize MPU6050
    while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) 
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

    mpu.calibrateGyro();  // Calibrate gyroscope
    mpu.setThreshold(3); // Set threshold sensitivity
  
  /*
    // Assign initial values to min and max
    for (int i = 0; i < numPins; i++) {
      minValues[i] = analogRead(pins[i]);
      maxValues[i] = analogRead(pins[i]);
    }
  */
}


void loop() {
    timer = millis();
    Vector normGyro = mpu.readNormalizeGyro();
    yaw = yaw + normGyro.ZAxis * timeStep;
    
    Serial.print(" Yaw = ");
    Serial.println(yaw);

    // Wait for full timeStep period
    delay(abs((timeStep*1000) - (millis() - timer)));
    
    
    for (int i = 0; i < 5; i++) {
      analogValue[i] = analogRead(analogPin[i]);
      //Serial.print(analogValue[i]);
      //Serial.print("\t");
    } 
    Serial.println("");

    path(0, 6);
    path(6, 0);
    //path(0, 6);
    //path(6, 0);
}
