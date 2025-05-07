#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// WiFi credentials
// const char* ssid = "TP-Link_56A7";
// const char* password = "A378904@";

const char* ssid = "jwaling's Galaxy A22 5G";
const char* password = "bxya0140";

// ThingSpeak settings
const char* channelID = "2933872";  // ThingSpeak channel ID
const char* readAPIKey = "BWQO64N4UII54EOV";  // Read API Key

// Variables for traffic light status
int trafficLightState = -1;  // -1 = unknown, 0 = Green, 1 = Yellow, 2 = Red
int remainingTime = 0;       // Remaining time in seconds
unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 5000;  // Check every 5 seconds

// Traffic light location (example coordinates)
float trafficLightLat = 12.9716;
float trafficLightLon = 77.5946;

// Variables for vehicle
float vehicleSpeed = 0;        // Current speed in km/h
float vehicleDistance = 500;   // Distance to traffic light in meters (you'll update this with GPS)

// Display/Output pins
const int speedoPin = 32;      // Pin for displaying recommended speed
const int buzzerPin = 33;      // Optional buzzer for alerts

void setup() {
  Serial.begin(115200);
  
  // Initialize output pins
  pinMode(speedoPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  
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
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update vehicle distance to traffic light (in a real system, this would come from GPS)
  // For testing, let's simulate approaching the traffic light
  if (vehicleDistance > 0) {
    // Assuming current speed is 30 km/h = 8.33 m/s
    float simulatedSpeed = 30;  // km/h
    float metersPerSecond = simulatedSpeed / 3.6;
    
    // Calculate how far we've moved since last cycle (assuming 100ms loop time)
    vehicleDistance -= (metersPerSecond * 0.1);
    if (vehicleDistance < 0) vehicleDistance = 0;
  }
  
  // Check traffic light status periodically
  if (currentTime - lastCheckTime >= checkInterval) {
    getTrafficLightStatus();
    lastCheckTime = currentTime;
  }
  
  // Calculate optimal speed based on traffic light status and distance
  float optimalSpeed = calculateOptimalSpeed();
  
  // Display optimal speed
  Serial.print("Distance to traffic light: ");
  Serial.print(vehicleDistance);
  Serial.println(" meters");
  
  Serial.print("Traffic light state: ");
  if (trafficLightState == 0) Serial.println("GREEN");
  else if (trafficLightState == 1) Serial.println("YELLOW");
  else if (trafficLightState == 2) Serial.println("RED");
  else Serial.println("UNKNOWN");
  
  Serial.print("Remaining time: ");
  Serial.print(remainingTime);
  Serial.println(" seconds");
  
  Serial.print("Recommended speed: ");
  Serial.print(optimalSpeed);
  Serial.println(" km/h");
  Serial.println("-------------------");
  
  // Small delay
  delay(100);
}

void getTrafficLightStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Reconnecting...");
    WiFi.reconnect();
    return;
  }
  
  HTTPClient http;
  String url = "https://api.thingspeak.com/channels/";
  url += channelID;
  url += "/feeds/last.json?api_key=";
  url += readAPIKey;
  
  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    String payload = http.getString();
    
    // Parse JSON response
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      trafficLightState = doc["field1"].as<int>();
      remainingTime = doc["field2"].as<int>();
      
      Serial.println("Traffic light data received:");
      Serial.print("State: ");
      Serial.print(trafficLightState);
      Serial.print(", Remaining time: ");
      Serial.println(remainingTime);
    } else {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.print("HTTP GET failed, error code: ");
    Serial.println(httpCode);
  }
  
  http.end();
}

float calculateOptimalSpeed() {
  // If we're at the traffic light or don't have valid data, return 0
  if (vehicleDistance <= 10 || trafficLightState < 0 || remainingTime <= 0) {
    return 0.0;
  }
  
  float optimalSpeed = 0.0;
  
  // Calculate time needed to reach the traffic light at different speeds
  if (trafficLightState == 2) {  // Red light
    // If red, aim to arrive when it turns green
    // Calculate speed to arrive just as light turns green
    optimalSpeed = (vehicleDistance / remainingTime) * 3.6;  // Convert m/s to km/h
    
    // Cap speed at reasonable limits
    if (optimalSpeed < 5.0) {
      // Too slow, better to stop
      optimalSpeed = 0.0;
    } else if (optimalSpeed > 60.0) {
      // Too fast, cap at max speed limit
      optimalSpeed = 60.0;
    }
  } else if (trafficLightState == 0) {  // Green light
    // If green, calculate if we can make it before it turns yellow
    if (vehicleDistance / (remainingTime + 5) * 3.6 <= 60.0) {
      // We can make it at a reasonable speed
      optimalSpeed = min(60.0, vehicleDistance / remainingTime * 3.6);
    } else {
      // Can't make it before yellow, slow down
      optimalSpeed = min(30.0, vehicleDistance / (remainingTime + 35) * 3.6);
    }
  } else if (trafficLightState == 1) {  // Yellow light
    // If yellow, calculate if we can make it before it turns red
    if (vehicleDistance / remainingTime * 3.6 <= 60.0 && vehicleDistance <= 50) {
      // Close enough and can make it at a reasonable speed
      optimalSpeed = min(60.0, vehicleDistance / remainingTime * 3.6);
    } else {
      // Better slow down for the upcoming red
      optimalSpeed = min(20.0, vehicleDistance / (remainingTime + 30) * 3.6);
    }
  }
  
  return optimalSpeed;
}