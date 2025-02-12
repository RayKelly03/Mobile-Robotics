#define BUFSIZE 512
#include <SPI.h>
#include <WiFi.h>
//#include <WiFiClient.h>
#include <WiFiServer.h>
// wifi details
char ssid[] = "iot";
char password[] = "snappe78certify";
String postBody;
String position;
String r;
String s;
int bk=0;
int cp; //current position
char server[] = "3.250.38.184";
int port = 8000;
WiFiClient client;

int route[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; // Initialize with -1
int routeSize = 0;  // Number of valid stops

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

bool connect() {
  Serial.println("Connecting to server");
  if (!client.connect(server, port)) {
    Serial.println("error connecting to server");
    return false;
  }
  Serial.println("Connected to server");
  return true;
}


// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  delay(1000);
  connectToWiFi();
  postBody="position=";
  position="0";
  Serial.println(position);
  postBody += position;
  Serial.println(postBody);
}

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


void getRoute() {
  connect(); 
  client.println("GET /api/getRoute/lkim7619 HTTP/1.1");
  client.println("Host: 3.250.38.184");
  client.println("Connection: close");
  client.println();
  r = readResponse();
  s = getResponseBody(r);
  Serial.print("Full Route:");
  Serial.println(s);
  
}

void convertArray(String data) {
  routeSize = 0;  
  int startIndex = 0, endIndex;

  while ((endIndex = data.indexOf(',', startIndex)) != -1 && routeSize < 10) {
    route[routeSize++] = data.substring(startIndex, endIndex).toInt();
    startIndex = endIndex + 1;
  }

  if (startIndex < data.length() && routeSize < 10) {
    route[routeSize++] = data.substring(startIndex).toInt();
  }

  for (int i = routeSize; i < 10; i++) {
    route[i] = -1;
  }
void loop() {
  /*
  //my code
  connect();
  client.println("POST /api/arrived/lkim7619 HTTP/1.1");
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println("Connection: close");
  client.println();

  // send post body
  client.println(postBody);
  r = readResponse();
  //getStatusCode(r);
  s = getResponseBody(r);
  Serial.println(r);
  client.stop();
  position = s;
  postBody = "position=";
  postBody += position;
  Serial.println(postBody);
  cp=position.toInt();
  */
  getRoute();
  delay(2000);
  convertArray(s);  // Convert to integer array

  Serial.print("Route as Array: ");
  for (int i = 0; i < 10; i++) {
    Serial.print(route[i]);
    if (i < 10 - 1) Serial.print(", ");
  }
  Serial.println();
  delay(15000);
}
