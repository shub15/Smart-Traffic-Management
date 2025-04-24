#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

// WiFi credentials
const char* ssid = "TP-Link_56A7";
const char* password = "A378904@";

// ThingSpeak settings
String apiKey = "CRYP45VHART8GYHV";  // Write API Key
const char* server = "api.thingspeak.com";

// Traffic light pins
const int redPin = 25;    // GPIO pin for red light
const int yellowPin = 26; // GPIO pin for yellow light
const int greenPin = 27;  // GPIO pin for green light

// Traffic light timing (in milliseconds)
const unsigned long greenTime = 30000;  // 30 seconds
const unsigned long yellowTime = 5000;  // 5 seconds
const unsigned long redTime = 30000;    // 30 seconds

// Variables to track state
int currentState = 0;  // 0 = Green, 1 = Yellow, 2 = Red
unsigned long stateStartTime = 0;
unsigned long lastUploadTime = 0;
const unsigned long uploadInterval = 5000;  // Upload every 5 seconds

void setup() {
  Serial.begin(115200);
  
  // Initialize traffic light pins
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  
  // Turn all lights off initially
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Start with green light
  setTrafficLight(0);
  stateStartTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long stateElapsedTime = currentTime - stateStartTime;
  
  // Check if it's time to change state
  if (currentState == 0 && stateElapsedTime >= greenTime) {
    // Change from Green to Yellow
    setTrafficLight(1);
    stateStartTime = currentTime;
  }
  else if (currentState == 1 && stateElapsedTime >= yellowTime) {
    // Change from Yellow to Red
    setTrafficLight(2);
    stateStartTime = currentTime;
  }
  else if (currentState == 2 && stateElapsedTime >= redTime) {
    // Change from Red to Green
    setTrafficLight(0);
    stateStartTime = currentTime;
  }
  
  // Upload data to ThingSpeak periodically
  if (currentTime - lastUploadTime >= uploadInterval) {
    uploadTrafficLightStatus();
    lastUploadTime = currentTime;
  }
}

void setTrafficLight(int state) {
  currentState = state;
  
  // Set lights based on state
  digitalWrite(greenPin, state == 0 ? HIGH : LOW);
  digitalWrite(yellowPin, state == 1 ? HIGH : LOW);
  digitalWrite(redPin, state == 2 ? HIGH : LOW);
  
  Serial.print("Traffic light changed to: ");
  if (state == 0) Serial.println("GREEN");
  else if (state == 1) Serial.println("YELLOW");
  else if (state == 2) Serial.println("RED");
}

void uploadTrafficLightStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Reconnecting...");
    WiFi.reconnect();
    return;
  }
  
  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=" + apiKey;
  
  // Calculate remaining time in current state
  unsigned long currentTime = millis();
  unsigned long stateElapsedTime = currentTime - stateStartTime;
  unsigned long remainingTime = 0;
  
  if (currentState == 0) {
    remainingTime = (greenTime > stateElapsedTime) ? (greenTime - stateElapsedTime) / 1000 : 0;
  } else if (currentState == 1) {
    remainingTime = (yellowTime > stateElapsedTime) ? (yellowTime - stateElapsedTime) / 1000 : 0;
  } else if (currentState == 2) {
    remainingTime = (redTime > stateElapsedTime) ? (redTime - stateElapsedTime) / 1000 : 0;
  }
  
  // ThingSpeak fields
  // Field 1: Traffic light state (0=Green, 1=Yellow, 2=Red)
  // Field 2: Remaining time in seconds
  url += "&field1=" + String(currentState) + "&field2=" + String(remainingTime);
  
  Serial.print("Uploading to ThingSpeak: State=");
  Serial.print(currentState);
  Serial.print(", Remaining Time=");
  Serial.print(remainingTime);
  Serial.println(" seconds");
  
  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("HTTP Response: " + payload);
  } else {
    Serial.println("HTTP GET failed");
  }
  
  http.end();
}