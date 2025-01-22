#include <WiFi.h>

// WiFi details
char ssid[] = "iot";
char password[] = "pashalik20pedipulation";

WiFiClient client;

// Server details
char server[] = "3.250.38.184";
int port = 8000;

// Function to connect to WiFi
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
  Serial.println(" Connected");
  Serial.print("Obtaining IP address");
  Serial.flush();

  while (WiFi.localIP() == INADDR_NONE) {
    Serial.print(".");
    Serial.flush();
    delay(300);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Function to connect to the server
bool connectServer() {
  if (!client.connect(server, port)) {
    Serial.println("Error connecting to server");
    return false;
  }
  Serial.println("Connected to server");
  return true;
}

// Function to read HTTP response
String readResponse() {
  #define BUFSIZE 512
  char buffer[BUFSIZE];
  memset(buffer, 0, BUFSIZE);
  client.readBytes(buffer, BUFSIZE);
  String response(buffer);
  return response;
}

// Function to extract HTTP status code
int getStatusCode(String& response) {
  String code = response.substring(9, 12); // HTTP status code starts at index 9
  return code.toInt();
}

// Function to extract HTTP response body
String getResponseBody(String& response) {
  int split = response.indexOf("\r\n\r\n");
  if (split != -1) {
    String body = response.substring(split + 4);
    body.trim();
    return body;
  }
  return "";
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  connectToWiFi();
  
}

void loop() {
  connectServer();
  // Replace 'position' with the actual value or logic to determine the position
  int position = 0; // Example value
  String postBody = "position=" + String(position);

  // Send POST request and headers
  client.println("POST /api/arrived/lkim7619 HTTP/1.1");
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println("Connection: close");
  client.println();
  client.println(postBody);

  // Read HTTP response
  String response = readResponse();
  int statusCode = getStatus  Code(response);

  if (statusCode == 200) {
    // Success, process the response body
    String body = getResponseBody(response);
    if (!body.equals("undefined")) {
      int destination = body.toInt();
      Serial.print("Destination: ");
      Serial.println(destination);
    }
  } else {
    Serial.print("HTTP Error Code: ");
    Serial.println(statusCode);
  }

  // Disconnect from the server
  client.stop();

  // Delay before the next loop iteration
  delay(1000);
}
