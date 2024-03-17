#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h>

// Define pins used for software serial communication with the GPS module
const int gpsRX = 2;
const int gpsTX = 3;

// Create a software serial object to communicate with the GPS module
SoftwareSerial gpsSerial(gpsRX, gpsTX);

// Create a TinyGPSPlus object
TinyGPSPlus gps;


#define SDA_PIN 4
#define SCL_PIN 7

//Create a Compass Object
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  // Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(9600);   // Start serial communication with computer
  gpsSerial.begin(9600); // Start serial communication with GPS module

  if(!mag.begin())
    {
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }
  
}


int degToDelay(int degrees, int refDeg = 90, int refDelay = 500) {
  if (degrees <= 0 || refDeg <= 0 || refDelay <= 0) {
    return 0; // Handle invalid inputs
  }
  return degrees * refDelay / refDeg;
}



void loop() {
 
  // Read data from the GPS module
  if (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      // If a GPS sentence is parsed successfully
      if (gps.location.isValid()) {
        // If location data is valid
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        Serial.println();
        Serial.print("Longitude: ");
        Serial.print(gps.location.lng(), 6);
        Serial.println();

        double targetLatitude = 9.591567;      // Hardcoded for sample reference
        double targetLongitude = 76.522156;    // Hardcoded for sample reference

        double distanceToTarget = gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLatitude, targetLongitude);
        double bearingToTarget = gps.courseTo(gps.location.lat(), gps.location.lng(), targetLatitude, targetLongitude);
        Serial.println(distanceToTarget);
        Serial.println(bearingToTarget);

        // sensors_event_t event; 
        // mag.getEvent(&event);

        // // Calculate heading (raw values may need scaling)
        // double heading = atan2(event.magnetic.y, event.magnetic.x);

        // // Normalize to 0-360 degrees: 
        // float headingDegrees = heading * 180/M_PI; 
        // // Serial.println(heading);
        // Serial.println(headingDegrees);

        delay(500);
      
      }
    }
  }
}