#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <MPU6050_tockn.h>  // Library for MPU6050 IMU

// WiFi credentials
const char* ssid = "note9";
const char* password = "12345678";

// ThingSpeak settings
const char* channelID = "2933872";  // ThingSpeak channel ID
const char* readAPIKey = "BWQO64N4UII54EOV";  // Read API Key

// GPS module connection
#define GPS_SERIAL_NUM 1
#define GPS_RX_PIN 16   // GPS TX connects to this pin
#define GPS_TX_PIN 17   // GPS RX connects to this pin
#define GPS_BAUD 9600   // Most GPS modules use 9600 baud rate

// IMU setup
MPU6050 mpu6050(Wire);
float accelX, accelY, accelZ;  // Acceleration in m/s²
float gyroX, gyroY, gyroZ;     // Angular velocity in degrees/s
bool imuCalibrated = false;

// Initialize the GPS library
TinyGPSPlus gps;
HardwareSerial GPSSerial(GPS_SERIAL_NUM);

// Traffic light location (fixed coordinates)
float trafficLightLat = 12.9716;
float trafficLightLon = 77.5946;

// Variables for traffic light status
int trafficLightState = -1;  // -1 = unknown, 0 = Green, 1 = Yellow, 2 = Red
int remainingTime = 0;       // Remaining time in seconds
unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 5000;  // Check every 5 seconds

// Variables for vehicle
float vehicleLat = 0.0;      // Current latitude
float vehicleLon = 0.0;      // Current longitude
float vehicleSpeed = 0.0;    // Current speed in km/h
float vehicleDistance = 0.0; // Distance to traffic light in meters
float vehicleAcceleration = 0.0; // Current acceleration in m/s²
unsigned long lastGPSUpdate = 0;
unsigned long lastIMUUpdate = 0;

// Kalman filter variables for better speed estimation
float estimatedSpeed = 0.0;  // Filtered speed estimate
float speedVariance = 5.0;   // Initial variance

// Display/Output pins
const int speedoPin = 32;    // Pin for displaying recommended speed
const int buzzerPin = 33;    // Optional buzzer for alerts

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for IMU
  Wire.begin();
  mpu6050.begin();
  Serial.println("Calibrating IMU...");
  mpu6050.calcGyroOffsets(true);  // Calibrate gyroscope
  imuCalibrated = true;
  Serial.println("IMU calibration done!");
  
  // Initialize GPS serial communication
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
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
  
  Serial.println("Waiting for GPS to initialize...");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update GPS data
  updateGPS();
  
  // Update IMU data
  updateIMU();
  
  // Check traffic light status periodically
  if (currentTime - lastCheckTime >= checkInterval) {
    getTrafficLightStatus();
    lastCheckTime = currentTime;
  }
  
  // Fuse GPS and IMU data for better speed estimation
  fuseSpeedData();
  
  // Calculate optimal speed based on traffic light status and distance
  float optimalSpeed = calculateOptimalSpeed();
  
  // Display information
  // if (currentTime - lastGPSUpdate <= 5000) {  // Only show if GPS data is recent
    Serial.print("Vehicle position: ");
    Serial.print(vehicleLat, 6);
    Serial.print(", ");
    Serial.println(vehicleLon, 6);
    
    Serial.print("GPS speed: ");
    Serial.print(vehicleSpeed);
    Serial.print(" km/h, Estimated speed: ");
    Serial.print(estimatedSpeed);
    Serial.println(" km/h");
    
    Serial.print("Acceleration: ");
    Serial.print(vehicleAcceleration);
    Serial.println(" m/s²");
    
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
  // }
  
  // Small delay
  // delay(100);
}

void updateGPS() {
  // Read data from GPS
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      // If we have a valid location
      if (gps.location.isValid()) {
        vehicleLat = gps.location.lat();
        vehicleLon = gps.location.lng();
        
        // Update speed if available (in km/h)
        if (gps.speed.isValid()) {
          vehicleSpeed = gps.speed.kmph();
        }
        
        // Calculate distance to traffic light (in meters)
        vehicleDistance = calculateDistance(vehicleLat, vehicleLon, trafficLightLat, trafficLightLon);
        
        lastGPSUpdate = millis();
      }
    }
  }
  
  // Check if GPS is connected
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS data received. Check wiring.");
  }
}

void updateIMU() {
  if (!imuCalibrated) return;
  
  // Update IMU readings
  mpu6050.update();
  
  // Get acceleration data (in m/s²)
  accelX = mpu6050.getAccX() * 9.81;  // Convert to m/s²
  accelY = mpu6050.getAccY() * 9.81;
  accelZ = mpu6050.getAccZ() * 9.81;
  
  // Get gyroscope data (in degrees/s)
  gyroX = mpu6050.getGyroX();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();
  
  // Calculate forward acceleration (assuming X is forward direction)
  // This is simplified and would need to be adjusted based on IMU orientation
  vehicleAcceleration = accelX;
  
  lastIMUUpdate = millis();
}

void fuseSpeedData() {
  // Simple Kalman filter for speed estimation
  // Only update if we have recent IMU data
  if (millis() - lastIMUUpdate > 1000) return;
  
  // Time since last update in seconds
  float dt = (millis() - lastIMUUpdate) / 1000.0;
  
  // Prediction step
  float predictedSpeed = estimatedSpeed + vehicleAcceleration * dt;
  
  // Update step only if GPS data is valid
  if (millis() - lastGPSUpdate < 5000) {
    float kalmanGain = speedVariance / (speedVariance + 5.0);  // Assuming GPS variance is 5.0
    estimatedSpeed = predictedSpeed + kalmanGain * (vehicleSpeed - predictedSpeed);
    speedVariance = (1 - kalmanGain) * speedVariance;
  } else {
    // Just use the prediction if no GPS update
    estimatedSpeed = predictedSpeed;
    speedVariance += 0.1;  // Increase uncertainty over time
  }
  
  // Ensure speed is non-negative
  if (estimatedSpeed < 0) estimatedSpeed = 0;
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
  // If we don't have valid data or are at the traffic light, return 0
  if (vehicleDistance <= 10 || trafficLightState < 0 || remainingTime <= 0) {
    return 0.0;
  }
  
  float optimalSpeed = 0.0;
  
  // Calculate time needed to reach the traffic light at different speeds
  if (trafficLightState == 2) {  // Red light
    // If red, aim to arrive when it turns green
    // Calculate speed to arrive just as light turns green
    optimalSpeed = (vehicleDistance / remainingTime) * 3.6;  // Convert m/s to km/h
    
    // Use current speed and acceleration to check if speed is achievable
    float currentSpeed = estimatedSpeed;
    float timeToReach = (optimalSpeed - currentSpeed) / (vehicleAcceleration * 3.6);
    
    // Cap speed at reasonable limits
    if (optimalSpeed < 5.0 || timeToReach < 0) {
      // Too slow or requires deceleration, better to stop
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

// Calculate distance between two GPS coordinates using the Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000; // Earth radius in meters
  
  float latRad1 = radians(lat1);
  float latRad2 = radians(lat2);
  float lonRad1 = radians(lon1);
  float lonRad2 = radians(lon2);
  
  float diffLat = latRad2 - latRad1;
  float diffLon = lonRad2 - lonRad1;
  
  float a = sin(diffLat/2) * sin(diffLat/2) + 
            cos(latRad1) * cos(latRad2) * 
            sin(diffLon/2) * sin(diffLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return R * c; // Distance in meters
}