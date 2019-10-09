/*--------------------------
 * Serial Message Map
 * A -> large X, Y acceleration
 * B -> Vehicle topple
 */
#include "I2Cdev.h"
#include <NeoSWSerial.h>
#include<TinyGPS++.h>
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

TinyGPSPlus gps;
NeoSWSerial ss(4,3);

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

const int AccXMax = 3;
const int AccYMax = 3;
const int maxRoll = 500;
const int maxPitch = 500;

int strWheel = 0;
 
void cancelEmergency();

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
    ss.begin(9600);

    accelgyro.initialize();

    pinMode(13,OUTPUT);
    digitalWrite(13,LOW);
}

void loop() {
 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float AccX, AccY, AccZ;
    float latitude, longitude, speedGPS;
    
    AccX = (float)ax/2048;
    AccY = (float)ay/2048;
    //AccZ = (float)az/2048;
    
        Serial.print(AccX,2); Serial.print(',');
        Serial.print(AccY,2); Serial.print(',');
        Serial.print((float)gx,2); Serial.print(',');               
        Serial.print((float)gy,2);  Serial.print(','); 
        
        
    strWheel = analogRead(A2);
        //Serial.println(strWheel);

    while (ss.available())
      gps.encode(ss.read());    

    latitude = gps.location.lat();
    longitude = gps.location.lng();
    speedGPS = gps.speed.kmph();
        Serial.print(1.123456,6); Serial.print(',');
        Serial.print(2.123456,6); Serial.print(',');
        Serial.print(3.123456,6); Serial.print(',');

    if(AccX < AccXMax || AccY > AccYMax)
    {
      Serial.println('A');         
    }
  delay(1200);    
}

void cancelEmergency()
{
  int startTime = millis();
  int currentTime = 0;
  char t;
  int flag = 0;
  while(currentTime - startTime < 5000)
  { 
    if(Serial.available())
    {
      t = Serial.read();
      flag = 1;
      break; 
    }
    currentTime = millis();
  }
  if(flag==0)
    sendEmergencyMsg();
}

void sendEmergencyMsg()
{
  digitalWrite(13,HIGH);
  //GSM Module Message
}

