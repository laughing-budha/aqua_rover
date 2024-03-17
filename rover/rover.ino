

#include <Servo.h>     

Servo myservo;        

#define motor11 9             
#define motor12 8

#define motor21 12           
#define motor22 13

#define enable11 10          
#define enable22 11         


#define echo  6              
#define Trigg 5   
#define echo2  4              
#define Trigg2 3    
#define echo3  2              
#define Trigg3 1              

long obstacle_0,obstacle_90,obstacle_180;  
void Reverse(void);             
void Right(void);              
void Left(void);                
void Forward(void);             
void Disable_Motor(void);       
void Enable_Motor(void);        
long Read_Ultrasonic_sensor_Value(void); 
void ScanSurrounding(void);     

void setup() {
  
  Serial.begin(9600);          
  pinMode(motor11, OUTPUT);    
  pinMode(motor12, OUTPUT);

  pinMode(motor21, OUTPUT);
  pinMode(motor22, OUTPUT);

  pinMode(enable11, OUTPUT);
  pinMode(enable22, OUTPUT);

  pinMode(echo, INPUT);        
  pinMode(Trigg, OUTPUT);
  pinMode(echo2, INPUT);        
  pinMode(Trigg2, OUTPUT);
  pinMode(echo3, INPUT);        
  pinMode(Trigg3, OUTPUT);

  Serial.println(" Obstacle detector Robot is about to start ");
  myservo.attach(2);      
  delay(50);              
  myservo.write(90);      
  delay(2000);

}


void loop() 
{
  ScanSurrounding();
  while(1)
  {   
    if(obstacle_90 > 150 && Read_Brake_sensor_Value())     
    { 
      Serial.println(" No obstacle detected at 90 deg  ");
      myservo.write(90);
      delay(500);
      Forward();
      Enable_Motor();
      while(obstacle_90 > 80 && Read_Brake_sensor_Value())    
      {
        obstacle_90 = Read_Ultrasonic_sensor_Value();
        Enable_Motor();
      } 
      Disable_Motor(); 
      Reverse();
      Enable_Motor();
      delay(50);  
      Disable_Motor();
    } 
    else if( (obstacle_0 > obstacle_180) && (obstacle_0 > 100)    ) 
    { 
      Serial.println(" No obstacle detected at right side, so turn right  ");
      Right();
      Enable_Motor();
      delay(500);
      Disable_Motor();
    } 
    else if( (obstacle_0 < obstacle_180) && (obstacle_180 > 100)  )
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
      while( (obstacle_0 < 100) || (obstacle_180 < 100) )  
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
      delay(350);
      Disable_Motor();
    }
    ScanSurrounding(); 
  }
  
}




void ScanSurrounding(void)
{
  Serial.println("Scanning surrounding");
  myservo.write(0);
  delay(800);
  obstacle_0 = Read_Ultrasonic_sensor_Value();


  myservo.write(90);
  delay(800);
  obstacle_90 = Read_Ultrasonic_sensor_Value();
  

  myservo.write(180);
  delay(800);
  obstacle_180 = Read_Ultrasonic_sensor_Value();
}

bool Read_Brake_sensor_Value(void)     
{
  long dist1=0;
  digitalWrite(Trigg2, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigg2, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigg2, LOW);
  dist1 = (pulseIn(echo2, HIGH)/ 10);
  long dist2=0;
  digitalWrite(Trigg3, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigg3, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigg3, LOW);
  dist2 = (pulseIn(echo3, HIGH)/ 10);
  if (dist1 > 170)           
  {
    dist1 = 170;
  }
  if (dist2 > 170)           
  {
    dist2 = 170;
  }
  Serial.print("brake 1 Distance is  --> ");
  Serial.println(dist1); 
  Serial.print("brake 2 Distance is  --> ");
  Serial.println(dist2);   
  // delay(1000);
  if(dist1<80 || dist2 <80){
    return false;
  }
  else{
    return true;
  }
}



long Read_Ultrasonic_sensor_Value(void)     
{
  long dist=0;
  digitalWrite(Trigg, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigg, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigg, LOW);
  dist = (pulseIn(echo, HIGH)/ 10);
  if (dist > 170)           
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


void Forward(void)              
{
  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);

  digitalWrite(motor21, LOW);
  digitalWrite(motor22, HIGH);
}


void Right(void)                
{

  digitalWrite(motor11, LOW);
  digitalWrite(motor12, HIGH);

  digitalWrite(motor21, LOW);
  digitalWrite(motor22, HIGH);
}


void Left(void)               
{

  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);

  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
}


void Reverse(void)             
{
  digitalWrite(motor11, LOW);
  digitalWrite(motor12, HIGH);

  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
}
