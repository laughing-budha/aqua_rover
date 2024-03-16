// Header Files ----------------------------------------------------------------

#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>   // Header file for GPS Module.
#include <Servo.h>    // Header file for Servo Motor.
#include <Adafruit_HMC5883_U.h>


//Pin Setup -------------------------------------------------------------------

const int gpsRX = 0; 
const int gpsTX = 1; // software serial communication with the GPS module

#define motor11 9             //Right Motor pins
#define motor12 8

#define motor21 12            //Left Motor pins
#define motor22 13

#define enable11 10          //Right Motor enable pin 
#define enable22 11          //Left Motor enable pin 


#define echo  6              //Ultrasonic's echo pin 
#define Trigg 5              //Ultrasonic's Trigger pin


// Object Creation ------------------------------------------------------------

Servo myservo;        // Object for servo motor
SoftwareSerial gpsSerial(gpsRX, gpsTX); // Software serial object to communicate with the GPS module
TinyGPSPlus gps; // TinyGPSPlus object
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);  //Compass Object


// Functions ------------------------------------------------------------------

long obstacle_0,obstacle_90,obstacle_180;  // abstacle at 0-deg, 90-deg, 180-deg respectively.
void Reverse(void);             //Reverse()
void Right(void);               //Right() Direction
void Left(void);                //Left() Direction
void Forward(void);             //Forward() Direction
void Disable_Motor(void);       //Stop motors   
void Enable_Motor(void);        //Start motors
long Read_Ultrasonic_sensor_Value(void); //Get available Distance 
void ScanSurrounding(void);     // look for obstacle 


//Setup File ------------------------------------------------------------------

void setup() {
  
  Serial.begin(115200);          //Serial communication reference
  gpsSerial.begin(115200); // Start serial communication with GPS module
  if(!mag.begin())
    {
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }

  pinMode(motor11, OUTPUT);    //L298 Pin mode assigning
  pinMode(motor12, OUTPUT);

  pinMode(motor21, OUTPUT);
  pinMode(motor22, OUTPUT);

  pinMode(enable11, OUTPUT);
  pinMode(enable22, OUTPUT);

  pinMode(echo, INPUT);        //Ultrasonic pin
  pinMode(Trigg, OUTPUT);

  Serial.println(" Obstacle detector Robot is about to start ");
  myservo.attach(2);      // Servo motor's signal Pin
  delay(50);              // signal processing delay
  myservo.write(90);      // Move servo at 90 deg
  delay(2000);
  
 

}

// Loop File ---------------------------------------------------------------------------

void loop() 
{


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
      }
    }
  }

  double targetLatitude = 9.591567;
  double targetLongitude = 76.522156;

  double distanceToTarget = gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLatitude, targetLongitude);
  double bearingToTarget = gps.courseTo(gps.location.lat(), gps.location.lng(), targetLatitude, targetLongitude);
  
  sensors_event_t event; 
        mag.getEvent(&event);

        // Calculate heading (raw values may need scaling)
        double heading = atan2(event.magnetic.y, event.magnetic.x);

        // Normalize to 0-360 degrees: 
        float headingDegrees = heading * 180/M_PI; 
        Serial.println(heading);

        delay(500);

  while (distanceToTarget > 5) { 

  ScanSurrounding();
  while(1)
  {   
    if(obstacle_90 > 120)     // No obstacle is detected then move straight 
    { 
      Serial.println(" No obstacle detected at 90 deg  ");
      myservo.write(90);
      delay(500);
      Forward();
      Enable_Motor();
      while(obstacle_90 > 80)    // keep moving till obstacle is detected
      { 
        obstacle_90 = Read_Ultrasonic_sensor_Value();
        Enable_Motor();
      } 
      Disable_Motor(); 
      Reverse();                
      Enable_Motor();
      delay(50);
      Disable_Motor();          // stop motor on obstacle detection
    } 
    else if( (obstacle_0 > obstacle_180) && (obstacle_0 > 100)    ) // right side space is greater than left side space   and right side space is sufficient to turn right.
    { 
      Serial.println(" No obstacle detected at right side, so turn right  ");
      Right();
      Enable_Motor();
      delay(500);
      Disable_Motor();
    } 
    else if( (obstacle_0 < obstacle_180) && (obstacle_180 > 100)  )// left side space is greater than right side space   and left side space is sufficient to turn left.
    { 
      Serial.println(" No obstacle detected at left side, so turn left  ");
      Left();
      Enable_Motor();
      delay(500);
      Disable_Motor();
    } 
    else
    { 
      Serial.println(" Obstacle detected on both side ---> moving backward ");
      while( (obstacle_0 < 100) || (obstacle_180 < 100) ) // keep moving back and look for space available on right or left side
      {
        Reverse();                
        
        Enable_Motor();
        delay(500);
        Disable_Motor();
        ScanSurrounding();  
      }      
      if(obstacle_0 > 70)
      {
       Right(); 
      }
      else if(obstacle_180 > 70)
      {
        Left();
      }
      Enable_Motor();
      delay(500);
      Disable_Motor();
    }
    ScanSurrounding(); 
  }
  
}

Serial.println("Target Reached!");

distanceToTarget = gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLatitude, targetLongitude);
bearingToTarget = gps.courseTo(gps.location.lat(), gps.location.lng(), targetLatitude, targetLongitude);

}


void ScanSurrounding(void)
{
  Serial.println("Scanning surrounding");
  myservo.write(0);
  delay(800);
  obstacle_0 = Read_Ultrasonic_sensor_Value();

//  delay(500);
  myservo.write(90);
  delay(800);
  obstacle_90 = Read_Ultrasonic_sensor_Value();
  
//  delay(500);
  myservo.write(180);
  delay(800);
  obstacle_180 = Read_Ultrasonic_sensor_Value();
}





long Read_Ultrasonic_sensor_Value(void)     //Distance calculation
{
  long dist=0;
  digitalWrite(Trigg, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigg, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigg, LOW);
  dist = (pulseIn(echo, HIGH)/ 10);
  if (dist > 170)           // to avoide garbage values.
  {
    dist = 170;
  }
  Serial.print(" Distance is  --> ");
  Serial.println(dist);   
  return(dist);
}


void Enable_Motor(void)
{
  analogWrite(enable11, 255);
  analogWrite(enable22, 255);
}


void Disable_Motor(void)
{
  analogWrite(enable11, 0);
  analogWrite(enable22, 0);
}


void Forward(void)               //Forward() Direction
{
  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);

  digitalWrite(motor21, LOW);
  digitalWrite(motor22, HIGH);
}


void Right(void)                 //Left() Direction
{

  digitalWrite(motor11, LOW);
  digitalWrite(motor12, HIGH);

  digitalWrite(motor21, LOW);
  digitalWrite(motor22, HIGH);
}


void Left(void)                //Right() Direction
{

  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);

  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
}


void Reverse(void)             // Reverse()
{
  digitalWrite(motor11, LOW);
  digitalWrite(motor12, HIGH);

  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
}
