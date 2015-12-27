//GPS definitions
#include <TinyGPS++.h>
#include <math.h>

float dt , time_new , time_old = 0;
float angle_error , dis , dis_d;
double LAT , LNG , LAT_old , LNG_old ,LAT_old1 , LNG_old1  ;
int i = 0; 

double LAT_d1 = 29.0639190673828125 , LNG_d1 = 31.09538459777832031250;
double LAT_d2 = 29.0639190673828125 , LNG_d2 = 31.09538459777832031250;
double LAT_d3 = 29.0639190673828125 , LNG_d3 = 31.09538459777832031250;

double LAT_d = LAT_d1 , LNG_d = LAT_d1;


unsigned int start = 0;
TinyGPSPlus gps;



//   IMU definiations 
#include "Wire.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = {'$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
float yaw;

//  END OF IMU definiations :

float pwm1, pwm2, pwm3, pwm4 ;
float delta;
float yaw_req = 0;
float yaw_error , limit_iyaw , yaw_integral = 0 , u_integral ;
float nom;
float kp , ki ;




void setup() 
{

  
  
  //  IMU setup 
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Serial.begin(115200);    
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
				

  mpu.setXAccelOffset(-2239);  
  mpu.setYAccelOffset(792);
  mpu.setZAccelOffset(5328);
  mpu.setXGyroOffset(48);
  mpu.setYGyroOffset(-35);
  mpu.setZGyroOffset(39);

 if (devStatus == 0) {
        // turn on the DMP, now that it's ready
       Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
       Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  // END OF THE IMU setup 

  //GPS Serial1 
  Serial1.begin(9600);
  
pinMode(3, OUTPUT);
pinMode(4, OUTPUT);
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);
pinMode(53, OUTPUT);
pinMode(51, OUTPUT);
pinMode(49, OUTPUT);
pinMode(47, OUTPUT);
}

void loop() 
{ 

//IMU---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
if (!dmpReady) return; while (!mpuInterrupt && fifoCount < packetSize) { } mpuInterrupt = false;  mpuIntStatus = mpu.getIntStatus(); fifoCount = mpu.getFIFOCount(); if ((mpuIntStatus & 0x10) || fifoCount == 1024) { mpu.resetFIFO(); Serial.println(F("FIFO overflow!"));} else if (mpuIntStatus & 0x02){ while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); mpu.getFIFOBytes(fifoBuffer, packetSize); fifoCount -= packetSize; mpu.dmpGetQuaternion(&q, fifoBuffer);  mpu.dmpGetGravity(&gravity, &q); mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); yaw = ypr[0]*180/M_PI; } 
 //                           Serial.print(angle_error);
// Serial.print('\t');       Serial.print(dis_d);
 Serial.print('\t');       Serial.print(yaw);
Serial.print('\t');       Serial.print(LAT,20);
Serial.print('\t');       Serial.print(LNG,20);

 
 
 //GPS 

  if( Serial1.available() ) { gps.encode( Serial1.read() );}
  LAT = gps.location.lat();
  LNG = gps.location.lng();
  dis =  distance(LAT_old , LNG_old , LAT , LNG );
  
   // For 2.5 m we parse GPS data and report some key values
  if (dis >= 2.5)
  { 
dis_d =  distance( LAT , LNG  , LAT_d , LNG_d );
angle_error = angleToNorth(LAT , LNG ,  LAT_d , LNG_d ) - angleToNorth(LAT_old , LNG_old ,  LAT , LNG); 
yaw_req = (angle_error + yaw);

LAT_old = LAT; 
LNG_old = LNG; 
  }


 
 
 
 
//MOTORS------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

kp = 2.5 , ki = 5;   //kp * yaw_error = constant * cosd(yaw_error) , yaw_error = 45

 yaw_error = yaw_req - yaw;
 if (abs(yaw_error)<2)  yaw_error = 0;
  if(yaw_error >  180) yaw_error = yaw_error - 360;
  if(yaw_error < -180) yaw_error = yaw_error + 360;
 

 //yaw integration
 yaw_integral += yaw_error * dt;
 if(abs(yaw_error) > 180){ki = 0; yaw_integral = 0;}
 float limit_iyaw = 100;
 if (abs(yaw_integral)* ki <= limit_iyaw) {u_integral = ki * yaw_integral;}
 else if(yaw_integral * ki >  limit_iyaw) {u_integral =  limit_iyaw; yaw_integral =  limit_iyaw/ki; }
                                  else    {u_integral = -limit_iyaw; yaw_integral = -limit_iyaw/ki; } 
   
   //action                            
 delta = kp*(yaw_error) + u_integral;
 nom = 200 *cos(yaw_error*M_PI/180);
 if (abs(yaw_error) >90)  nom = 0;
 
 

 //allocation
 pwm1 = pwm3 = nom - delta; 
 pwm2 = pwm4 = nom + delta;  
if(dis_d <= 2.5 & LAT_d == LAT_d1){LAT_d = LAT_d2 ; LNG_d = LNG_d2;} 
if(dis_d <= 2.5 & LAT_d == LAT_d2){LAT_d = LAT_d3 ; LNG_d = LNG_d3;} 
if(dis_d <= 2.5 & LAT_d == LAT_d3){pwm1 = pwm2 = pwm3 = pwm4 = 0;} 

if (!gps.location.isValid()) {  Serial.print('\t'); Serial.print("invalid GPS connection");  pwm2 = pwm4 = pwm1 = pwm3 = 0;}  
 
 
 motor(pwm1, 53, 3);
 motor(pwm2, 51, 4);
 motor(-pwm3, 49, 5);
 motor(-pwm4, 47, 6);
 
 
  Serial.println();

}



int motor(float pwm, int dir, int velosity)
{
  if (pwm > 255)
  pwm = 255;
  if(pwm<-255)
  pwm = -255;
  
  if(pwm < 0)
  {
  pwm = abs(pwm);
  digitalWrite(dir, LOW);
  analogWrite(velosity, pwm);
  }
  else
  {
  digitalWrite(dir, HIGH);
  analogWrite(velosity, pwm);
  }
  return 0;
  } 
  
  
  
  float distance(double LAT1 ,double LNG1 ,double LAT2 ,double LNG2)  //m
{
LAT1 = LAT1 * M_PI / 180.0;
LAT2 = LAT2 * M_PI / 180.0;
LNG1 = LNG1 * M_PI / 180.0;
LNG2 = LNG2 * M_PI / 180.0;

float R = 6371000.0; // m
double a = pow(sin((LAT2 - LAT1)/2),2) + cos(LAT1) * cos(LAT2) * pow(sin((LNG2 - LNG1)/2),2);
double c = 2 * atan2(sqrt(a),sqrt(1 - a));

return R * c ; 
}


float angleToNorth(double LAT1 ,double LNG1 ,double LAT2 ,double LNG2)  //degree
{
LAT1 = LAT1 * M_PI / 180.0;
LAT2 = LAT2 * M_PI / 180.0;
LNG1 = LNG1 * M_PI / 180.0;
LNG2 = LNG2 * M_PI / 180.0;

float angle = atan2(sin(LNG2 - LNG1) * cos(LAT2) , cos(LAT1) * sin(LAT2) - sin(LAT1) * cos(LAT2) * cos(LNG2 - LNG1)); 

return angle * 180.0 / M_PI;
}

float angle_car(double LAT1 ,double LNG1 ,double LAT2 ,double LNG2)  //degree
{


float angle = atan2(LNG2 - LNG1 , LAT2 - LAT1); 

return angle * 180.0 / M_PI;
}

