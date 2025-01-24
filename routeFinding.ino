// Node A = Node 5, Node B = Node 6

class Graph {
  private: 
    int adjMatrix[7][7];
    
  public:
    Graph() {
      
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                adjMatrix[i][j] = 0;
            }
        }
    }

    void addEdge (unsigned int v, unsigned int w, int weight) {
      if (v < 7 && w < 7) {
        adjMatrix[v][w] = weight;
      }
      else {
            Serial.println("Error: Vertex out of bounds");
        }
    }


    void displayMatrix() {
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                Serial.print(adjMatrix[i][j]);
                Serial.print(" ");
            }
            Serial.println();
        }
    }
};


//void dijkstra()
void setup() {
    Serial.begin(9600);
    Graph g;
    g.addEdge(0, 5, 1);
    g.addEdge(0, 4, 1);
    g.addEdge(1, 5, 1); 
    g.addEdge(1, 6, 1); 
    g.addEdge(2, 5, 1);
    g.addEdge(2, 3, 1);
    g.addEdge(3, 6, 1); 
    g.addEdge(3, 2, 1);
    g.addEdge(4, 6, 1);
    g.addEdge(4, 0, 1);
    g.addEdge(5, 0, 1); 
    g.addEdge(5, 1, 1);
    g.addEdge(5, 2, 1);
    g.addEdge(6, 3, 1);
    g.addEdge(6, 4, 1); 
    g.addEdge(6, 5, 1);   


    Serial.println("Adjacency Matrix:\n");
    g.displayMatrix();
}

void loop() {
  // put your main code here, to run repeatedly:

}
