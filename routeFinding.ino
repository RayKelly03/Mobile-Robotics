// Node A = Node 6, Node B = Node 7

#include <limits.h>


int Inf = INT_MAX;

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
        int parent[8];    // Array of shortest path to all other nodes (acts as a lookup table)

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
        if (parent[j] == -1) {
            Serial.print(j);
            return;
        }
        printPath(parent, parent[j]);
        Serial.print(" -> ");
        Serial.print(j);
    }
};


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
    g.addEdge(6, 5, 1);   // Creates Adjacency Matrix in format : (Node A, Node B, distance between)

    Serial.println("Adjacency Matrix:");
    g.displayMatrix();
}

void loop() {
  // put your main code here, to run repeatedly:

}
