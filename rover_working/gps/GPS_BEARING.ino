#include <Arduino.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

const int gpsRX = 2;
const int gpsTX = 3;

SoftwareSerial gpsSerial(gpsRX, gpsTX);
TinyGPSPlus gps;

#define RADIUS_OF_EARTH 6371000 // Earth radius in meters

// Function to convert degrees to radians
double toRadians(double degree) {
  return degree * PI / 180.0;
}

// Function to convert radians to degrees
double toDegrees(double radian) {
  return radian * 180.0 / PI;
}

// Function to calculate distance between two coordinates using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);

  double a = pow(sin(dLat / 2), 2) + cos(toRadians(lat1)) * cos(toRadians(lat2)) * pow(sin(dLon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return RADIUS_OF_EARTH * c;
}

// Function to calculate initial bearing between two coordinates
double calculateInitialBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = toRadians(lon2 - lon1);

  double y = sin(dLon) * cos(toRadians(lat2));
  double x = cos(toRadians(lat1)) * sin(toRadians(lat2)) -
             sin(toRadians(lat1)) * cos(toRadians(lat2)) * cos(dLon);

  double bearing = atan2(y, x);

  // Convert bearing from radians to degrees
  bearing = toDegrees(bearing);

  // Ensure the bearing is in the range [0, 360)
  bearing = fmod(bearing + 360, 360);

  return bearing;
}

void setup() {
  Serial.begin(9600);   // Start serial communication with computer
  gpsSerial.begin(9600); // Start serial communication with GPS module
}

void loop() {

  if (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        double lat1 = gps.location.lat(); // Latitude of point 1
        double lon1 = gps.location.lng(); // Longitude of point 1

        Serial.print("Latitude: ");
        Serial.println(lat1, 6);
        Serial.print("Longitude: ");
        Serial.println(lon1, 6);

        // Coordinates of point 2
        double lat2 = 10.0879301; // Latitude of point 2 (e.g., Los Angeles)
        double lon2 = 77.0627041; // Longitude of point 2

        // Calculate distance and bearing
        double distance = calculateDistance(lat1, lon1, lat2, lon2);
        double bearing = calculateInitialBearing(lat1, lon1, lat2, lon2);

        // Output results
        Serial.print("Distance between the two points: ");
        Serial.print(distance);
        Serial.println(" meters");
        Serial.print("Bearing from point 1 to point 2: ");
        Serial.print(bearing);
        Serial.println(" degrees");
      }
    }
  }
}
