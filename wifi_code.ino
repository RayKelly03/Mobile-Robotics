#define BUFSIZE 512
#include <SPI.h>
#include <WiFi.h>
//#include <WiFiClient.h>
#include <WiFiServer.h>
// wifi details
char ssid[] = "Tp Link";
char password[] = "Knockmore123";
String postBody;
String position;
String r;
String s;
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
  String response(buffer);
  return response;
}
  
int getStatusCode(String& response) {
  String code = response.substring(9, 12);
  return code.toInt();
}
    
String getResponseBody(String& response) {
  int split = response.indexOf("\r\n\r\n");
  String body = response.substring(split+4, response.length());
  body.trim();
  return body;
}

// the loop routine runs over and over again continuously:
void loop() {
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
  Serial.println(s);
  client.stop();
  position = s;
  postBody = "position=";
  postBody += position;
  Serial.println(postBody);
  cp=position.toInt();
  
  if(position!="undefined"){
    // normal function
  }
  else{
    // stop code
    while (1) {}
  }
}
