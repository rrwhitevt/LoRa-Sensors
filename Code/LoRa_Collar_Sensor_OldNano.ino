/*
  LoRa_Sender_MQTT:
  Support Devices: LoRa Shield + Arduino 
  
  Require Library:
  https://github.com/sandeepmistry/arduino-LoRa 
  
  Example sketch showing how to send or a message base on ThingSpeak(https://thingspeak.com) MQTT format. 

  The End node will send out a string "<End_Node_ID>field1=${TEMPERATURE_VALUE}&field2=${HUMIDITY_VALUE}" to LG01/LG02 gateway. 

  When the LG01/LG02 gateway get the data, it will parse and forward the data to ThingSpeak via MQTT protocol. 

  modified Dec 26 2018
  by Dragino Technology Co., Limited <support@dragino.com>
*/
#include <SPI.h>
#include "MPU9250.h"
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <LoRa.h>
#include "LowPower.h"
#include "Statistic.h"

float tC, aX, aY, aZ, mX, mY, mZ, gX, gY, gZ, mag_x, mag_y, pitch, yaw, roll;
int count=0;
int device_id=1002; // ID of this End node
static const int RXPin = 5, TXPin = 5;
static const uint32_t GPSBaud = 9600;
Statistic Lat, Long, Pitch, Roll, Yaw, KPH, Temp;

//Initialize MPU 9250
MPU9250 IMU(Wire, 0x68);
int status;

// The TinyGPS++ object
TinyGPS gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(915000000)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSyncWord(0x34);

  IMU.begin();
  ss.begin(GPSBaud);

}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(count);

  //Motion Sensor Data
  IMU.readSensor();
  aX = IMU.getAccelX_mss();    //accelerometer X axis from IMU wire
  aY = IMU.getAccelY_mss();    //accelerometer Y axis from IMU wire  
  aZ = IMU.getAccelZ_mss();    //accelerometer Z axis from IMU wire
  tC = IMU.getTemperature_C(); //Temperature in C from IMU wire    
  mX = IMU.getMagX_uT();    //accelerometer X axis from IMU wire
  mY = IMU.getMagY_uT();    //accelerometer Y axis from IMU wire  
  mZ = IMU.getMagZ_uT();    //accelerometer Z axis from IMU wire
  pitch = 180*atan2(aX, sqrt(aY*aY+aZ*aZ))/PI;
  roll = 180*atan2(aY, sqrt(aX*aX+aZ*aZ))/PI;
  mag_x = mX*cos(pitch) - mY*sin(roll)*sin(pitch)+mZ*cos(roll)*sin(pitch);
  mag_y = mY*cos(roll) - mZ*sin(roll);
  yaw = 180*atan2(-mag_y,mag_x)/PI;
  Serial.print(tC);
  Serial.print(", ");  
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  
  //GPS Location Information
  float flat, flon;
  float kph;
  unsigned long age;
  smartdelay(1000);
  gps.f_get_position(&flat, &flon, &age);
  kph = gps.f_speed_kmph();
  Serial.print(flat, 6);
  Serial.print(", ");
  Serial.print(flon, 6);
  Serial.print("");   
  Serial.print(kph, 6);
  Serial.println("");   

  while(Lat.count() <= 60000) {
    Lat.add(flat); 
    Long.add(flon);
    KPH.add(kph);
    Pitch.add(pitch);
    Yaw.add(yaw);
    Roll.add(roll);
    Temp.add(tC);
  }
  
  // compose and send packet
  LoRa.beginPacket();
  LoRa.print("<");
  LoRa.print(device_id);
  LoRa.print(">field1=");
  LoRa.print(Lat.average(), 6);
  LoRa.print("&field2=");
  LoRa.print(Long.average(), 6); 
  LoRa.print("&field3=");
  LoRa.print(Temp.average(), 4); 
  LoRa.print("&field4=");
  LoRa.print(Pitch.average(), 4); 
  LoRa.print("&field5=");
  LoRa.print(Yaw.average(), 4); 
  LoRa.print("&field6=");
  LoRa.print(Roll.average(), 4); 
  LoRa.print("&field7=");
  LoRa.print(KPH.average()); 
  LoRa.print("&field8=");
  LoRa.print(device_id); 
 // LoRa.print(counter);
  LoRa.endPacket();
  count++;
  
  if(Lat.count()>=60) {
    Lat.clear();
    Long.clear();
    KPH.clear();
    Pitch.clear();
    Yaw.clear();
    Roll.clear();
    Temp.clear();
  }
  
}
