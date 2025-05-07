// Sharp sensor pin
const int sharpSensorPin = 34;  // Analog input pin

// Road detection parameters
const int sampleSize = 10;        // Number of readings to average
const int potholeTrigger = 2;     // Increase in distance (mm) to detect potholes
const int speedBreakTrigger = 2;  // Decrease in distance (mm) to detect speed breakers
const int readingDelay = 100;     // Delay between readings in ms

// Global variables
float vehicleSpeed = 0.0;            // Current vehicle speed in km/h
float recommendedSpeed = 0.0;        // Recommended speed in km/h
int distanceToSignal = 500;          // Assumed distance to signal in meters (would be from GPS)
unsigned long lastReadingTime = 0;   // Last sensor reading time
float distanceReadings[sampleSize];  // Array to store recent distance readings
int readingIndex = 0;
float avgDistance = 0;
float prevAvgDistance = 0;

float fixedDistance = 10;


void setup() {
  Serial.begin(115200);

  // Initialize sensor readings array
  for (int i = 0; i < sampleSize; i++) {
    distanceReadings[i] = 0;
  }

  Serial.println("Vehicle Unit Started");
  fixedDistance = readSharpSensor();
}

void loop() {
  detectRoadConditions();
  // delay(100);
}

float readSharpSensor() {
  // Read analog value from Sharp sensor
  int sensorValue = analogRead(sharpSensorPin);

  // Convert analog reading to distance in mm
  // Formula depends on the specific Sharp sensor model
  // This is for GP2Y0A21YK0F (10-80cm range)
  float distance = 12343.85 * pow(sensorValue, -1.15);

  return distance;
}

void detectRoadConditions() {
  // Read current distance from sensor
  float currentDistance = readSharpSensor();

  float distanceChange = currentDistance - prevAvgDistance;

  // Detect speed breaker (distance decreases significantly)
  if (-distanceChange < fixedDistance) {
    Serial.println("ALERT: Speed Breaker detected!");
  }
  // Detect pothole (distance increases significantly)
  else if (distanceChange > fixedDistance) {
    Serial.println("ALERT: Pothole detected!");
  } else {
    Serial.println(currentDistance);
  }

  prevAvgDistance = currentDistance;
}