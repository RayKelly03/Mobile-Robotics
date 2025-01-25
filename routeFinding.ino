// Node A = Node 6, Node B = Node 7

#include <limits.h>


int Inf = INT_MAX;
int prev = -1;
int next = -1;
float gyroAngle = 0;

int analogValue[5] = {0, 0, 0, 0, 0};  // Store sensor values
int analogPin[5] = {4, 5, 6, 7, 15};    // Sensor pins

int motor1PWM = 37;    // Left motor speed
int motor1Phase = 38;  // Left motor direction
int motor2PWM = 39;    // Right motor speed
int motor2Phase = 20;  // Right motor direction

int speed = 100;       // Forward speed
int turnSpeed = 50;    // Default turn speed
int sharpTurnSpeed = 100;  // Faster speed for sharper turns

void Forward() {
  digitalWrite(motor1Phase, LOW);  // Move both motors forward
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, speed);
  analogWrite(motor2PWM, speed);
}

void TurnLeftSmooth() {
  digitalWrite(motor1Phase, LOW);      // Slow down left motor
  digitalWrite(motor2Phase, LOW);      // Keep right motor at normal speed
  analogWrite(motor1PWM, turnSpeed);  // Sharp turn left
  analogWrite(motor2PWM, speed);       // Normal speed for right motor
}

void TurnRightSmooth() {
  digitalWrite(motor1Phase, LOW);      // Keep left motor at normal speed
  digitalWrite(motor2Phase, LOW);      // Slow down right motor
  analogWrite(motor1PWM, speed);       // Normal speed for left motor
  analogWrite(motor2PWM, turnSpeed);  // Sharp turn right
}

void TurnLeftSharp() {
  digitalWrite(motor1Phase, HIGH);      // Slow down left motor
  digitalWrite(motor2Phase, LOW);      // Keep right motor at normal speed
  analogWrite(motor1PWM, speed);  // Sharp turn left
  analogWrite(motor2PWM, speed);       // Normal speed for right motor
}

void TurnRightSharp() {
  digitalWrite(motor1Phase, LOW);      // Keep left motor at normal speed
  digitalWrite(motor2Phase, HIGH);      // Slow down right motor
  analogWrite(motor1PWM, speed);       // Normal speed for left motor
  analogWrite(motor2PWM, speed);  // Sharp turn right
}

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


    void dijkstra(int src) {
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
        printSolution(dist, parent, src);
    }

    // Print the distance and paths
    void printSolution(int dist[], int parent[], int src) {
        Serial.println("Vertex\tDistance\tPath");
        for (int i = 0; i < 8; i++) {
            Serial.print(i);
            Serial.print("\t");
            Serial.print(dist[i]);
            Serial.print("\t\t");
            printPath(parent, i);
            Serial.println();
        }
    }

    // Prints the shortest path - uses parent array
    void printPath(int parent[], int j) {
        int path[30];
        int k = 0;
        if (parent[j] == -1) {
            path[k++] = j;
            return;
        }
        printPath(parent, parent[j]);
    }
};

void followLine() {
  if (analogValue[2] < 300) {  // Center sensor detects white (on the line)
    Forward();
    Serial.println("Moving Forward");
  }
  
  else if (analogValue[1] < 300) {  // Left sensor detects white
    TurnLeftSmooth();
    Serial.println("Turning Left");
  }
  
  else if (analogValue[3] < 300) {  // Right sensor detects white
    TurnRightSmooth();
    Serial.println("Turning Right");
  }
  
  else if (analogValue[4] < 300) {  // Far-right sensor detects white
    TurnRightSharp();
    Serial.println("Adjusting Right (Far Right Sensor)");
  }
  
  else if (analogValue[0] < 300) {  // Far-left sensor detects white
    TurnLeftSharp();
    Serial.println("Adjusting Left (Far Left Sensor)");
  }
  
  else {
    //Stop();  // If no sensor detects white, stop the robot
    Serial.println("Stopped");
  }

  delay(20);  // Small delay for smoother loop
}



void path(int gyroAngle, int prev, int next) {
  if(next == 0 && prev == 6){
    //setGyroAng(-1);
    prev = 0;
    next = 1;
  
    while(analogValue[0], analogValue[1], analogValue[2], analogValue[3], analogValue[4] > 300) {
      followLine();
  
      //if (frontSensor > 300) {
        //removeEdgeRedirect();
      //}
    }
  }
  
}

void removeEdgeRedirect(int gyroAngle, int prev,int next, Graph g) {
  //setGyroAngle(gyroAngle + 180);
  while(analogValue[0], analogValue[1], analogValue[2], analogValue[3], analogValue[4] > 300) {
    followLine();
  }

  g.removeEdge(prev, next);
  g.dijkstra(prev);
  //path(prev, next);
  
}


void setup() {
    Serial.begin(9600);
    Graph g; // Creates Graph
    g.addEdge(0, 5, 1);
    g.addEdge(0, 4, 1);
    g.addEdge(1, 5, 1); 
    g.addEdge(1, 6, 1); 
    g.addEdge(2, 5, 1);
    g.addEdge(2, 3, 1);
    g.addEdge(3, 6, 1); 
    g.addEdge(4, 6, 1);
    g.addEdge(6, 5, 1);   // Creates Adjacency Matrix in format : addEdge(Node A, Node B, distance between)

    Serial.println("Adjacency Matrix:");
    g.displayMatrix();

    for (int i = 0; i < 5; i++) {
    pinMode(analogPin[i], INPUT);  // Sensor pins setup
  }
}

void loop() {
  for (int i = 0; i < 5; i++) {
    analogValue[i] = analogRead(analogPin[i]);
    Serial.print(analogValue[i]);
    Serial.print("\t");
    if (i == 4) {
      Serial.println("");  // New line after all readings
      delay(100);
    }
  }
}
