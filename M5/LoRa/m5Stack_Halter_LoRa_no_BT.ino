/*
VT SmartFarm Sensor Node Code For:
M5Stack with RMF95 radio
  MISO -> 19
  MOSI -> 23
  SCK -> 18
  NSS -> 5
  Reset -> 26
  DID0 -> 36
  3.3v -> 3.3v
  GND -> G
  ANA -> Not connected to m5
GPS
  3.3v -> 3.3v
  G -> G
  Data -> R2
Adafruit Seesaw Soil Moisture Probe
  GND -> G
  VIN -> 3.3v
  SDA -> 21
  SCL -> 22
APDS 9960
  GND -> G
  VCC -> 3.3v
  SDA -> SDA
  SCL -> SCL
*/


/* ____________________________________________________________________________
   Open Libraries
   ____________________________________________________________________________
*/

//From the M5Stack library 
#include <M5Stack.h>
#include <M5LoRa.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

//From the Tiny GPS++ Library
#include <TinyGPS++.h>

//From the arduino-geohash library
#include <arduino-geohash.h>

//From the Statistic library
#include "Statistic.h"

//From the Adafruit_seesaw library
#include "Adafruit_seesaw.h"

//From the Wire library
#include <Wire.h>

//From the SparkFun_APDS9960 Library
#include <SparkFun_APDS9960.h>


/* ____________________________________________________________________________
   Define Global Variables
   ____________________________________________________________________________
*/


static const uint32_t GPSBaud = 9600;
int device_id = 2003;

Statistic tmStat, axStat, ayStat, azStat, gxStat, gyStat, gzStat, mxStat, myStat, mzStat, stStat, scStat;
int interval = 30; //seconds to collect data over
int htz = 100; //sample collection interval in hertz
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
uint8_t proximity_data = 0;
float latitude; 
float longitude;

/* ____________________________________________________________________________
   Set Up Sensor Objects
   ____________________________________________________________________________
*/

TinyGPSPlus gps;
MPU9250 IMU;
Adafruit_seesaw sensor;
SparkFun_APDS9960 apds = SparkFun_APDS9960();


// The serial connection to the GPS device
HardwareSerial ss(2);

// Set up hasher
GeoHash hasher(17);


/* ____________________________________________________________________________
   Define Custom Functions
   ____________________________________________________________________________
*/


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


void setup()
{
//wake up the m5
  M5.begin();  

//initialize GPS
  ss.begin(GPSBaud);
  Serial.println("GPS Initialized");

//Initialize MPU9250
  Wire.begin();
  IMU.initMPU9250();
  Serial.println("IMU Initialized");

/*Initialize Soil Sensor
  if (!sensor.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(sensor.getVersion(), HEX);
  }
/*
//Initialize ADPS-9960
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }

    // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
  // Start running the APDS-9960 proximity sensor (no interrupts)
  if ( apds.enableProximitySensor(false) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
*/
  
  // Wait for initialization and calibration to finish
  delay(500);

//Initialize LoRa Radio
  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(); // default set CS, reset, IRQ pin
  Serial.println("LoRa Sender");

  // frequency in Hz (433E6, 866E6, 915E6)
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // LoRa.setSyncWord(0x69);
  Serial.println("LoRa init succeeded.");
  delay(2000);


//Apply power-saving adjustments to m5
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setBrightness(0);
  Serial.begin(115200);
}

void loop()
{
//Begin loading IMU and SS data into stats containers until inter*htz samples are collected  
  while(tmStat.count() < interval*htz){
  //Print IMU values
  IMU.readAccelData(IMU.accelCount);
  IMU.getAres();
  IMU.ax = (float)IMU.accelCount[0]*IMU.aRes; // - accelBias[0];
  IMU.ay = (float)IMU.accelCount[1]*IMU.aRes; // - accelBias[1];
  IMU.az = (float)IMU.accelCount[2]*IMU.aRes; // - accelBias[2];

  IMU.readGyroData(IMU.gyroCount);
  IMU.getGres();
  IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes; 
  IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes; 
  IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes; 
  
  IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
  IMU.getMres();  

  IMU.mx = (float)IMU.magCount[0]*IMU.mRes;
  IMU.my = (float)IMU.magCount[1]*IMU.mRes;
  IMU.mz = (float)IMU.magCount[2]*IMU.mRes;
  
  //Calculate quaternion
  IMU.updateTime();
  MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx*DEG_TO_RAD,
                         IMU.gy*DEG_TO_RAD, IMU.gz*DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);
                         
  // Print acceleration values in milligs!
  float ax = float(1000*IMU.ax);
  float ay = float(1000*IMU.ay);
  float az = float(1000*IMU.az);

  // Print gyro values in degree/sec
  float gx = float(IMU.gx);
  float gy = float(IMU.gy);
  float gz = float(IMU.gz);

  // Print mag values in degree/sec
  float mx = float(IMU.mx);
  float my = float(IMU.my);
  float mz = float(IMU.mz);

  IMU.tempCount = IMU.readTempData();  // Read the adc values
  // Temperature in degrees Centigrade
  IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;

  float tm = float(IMU.temperature);

/*
  float tempC = tempC = sensor.getTemp();
  uint16_t capread = sensor.touchRead(0);
  stStat.add(tempC); scStat.add(capread); 
*/
  tmStat.add(tm); axStat.add(ax); ayStat.add(ay); azStat.add(az); mxStat.add(mx); myStat.add(my); mzStat.add(mz); gxStat.add(gx); gyStat.add(gy); gzStat.add(gz);
  smartDelay(htz/1000);
  }

  String tmMean = String(tmStat.average());
  String axMean = String(axStat.average());
  String ayMean = String(ayStat.average());
  String azMean = String(azStat.average());
  String gxMean = String(gxStat.average());
  String gyMean = String(gyStat.average());
  String gzMean = String(gzStat.average());
  String mxMean = String(mxStat.average());
  String myMean = String(myStat.average());
  String mzMean = String(mzStat.average());

/*
  String stMean = String(stStat.average());
  String scMean = String(scStat.average());
*/
  
  //GPS Values for SD card/BT
  String nSat = String(gps.satellites.value(), 5);
  String Dop = String(gps.hdop.value(), 5);
  String DateM = String(gps.date.month());
  String DateD = String(gps.date.day());
  String DateY = String(gps.date.year());
  String TimeH = String(gps.time.hour());
  String TimeM = String(gps.time.minute());
  String TimeS = String(gps.time.second());
  String Alt = String(gps.altitude.meters(),2);
  String Vol = String(gps.speed.kmph(), 2);

  latitude = float(gps.location.lat());
  longitude = float(gps.location.lng());
  const char* geohash = hasher.encode(latitude, longitude);
  String GH = String(geohash);

/*Read the light levels (ambient, red, green, blue)
  String al = String(apds.readAmbientLight(ambient_light));
  String bl = String(apds.readRedLight(red_light));
  String rl = String(apds.readBlueLight(blue_light));
  String gl = String(apds.readGreenLight(green_light));
  
 
// Read the proximity value
  String px = String(apds.readProximity(proximity_data));
*/

//Convert Device ID to String
  String sensorname = String(device_id);

  //Create Outstring to append to SD card
//  String GPSOut = sensorname+",GPS,"+DateM+","+DateD+","+DateY+","+TimeH+","+TimeM+","+TimeS+","+nSat+","+Dop+","+GH+","+Alt+","+Vol+","+stMean+","+scMean;
  String GPSOut = sensorname+",GPS,"+DateM+","+DateD+","+DateY+","+TimeH+","+TimeM+","+TimeS+","+nSat+","+Dop+","+GH+","+Alt+","+Vol;
  String IMUOut = sensorname+",IMU,"+DateM+","+DateD+","+DateY+","+TimeH+","+TimeM+","+TimeS+","+tmMean+","+axMean+","+ayMean+","+azMean+","+gxMean+","+gyMean+","+gzMean+","+mxMean+","+myMean+","+mzMean;

  //Send packet out through LoRa
  LoRa.setTxPower(10);
  LoRa.beginPacket();
  LoRa.print("<");
  LoRa.print(device_id);
  LoRa.print(">");
  LoRa.print(GPSOut);
  LoRa.endPacket();

  //print lora string
  Serial.print("<");
  Serial.print(device_id);
  Serial.print(">");
  Serial.println(GPSOut);


  delay(10000);

  
  LoRa.beginPacket();
  LoRa.print("<");
  LoRa.print(device_id);
  LoRa.print(">");
  LoRa.print(IMUOut);
  LoRa.endPacket();

  //print lora string
  Serial.print("<");
  Serial.print(device_id);
  Serial.print(">");
  Serial.println(IMUOut);

  
  tmStat.clear();
  axStat.clear();
  ayStat.clear();
  azStat.clear();
  gxStat.clear();
  gyStat.clear();
  gzStat.clear();
  mxStat.clear();
  myStat.clear();
  mzStat.clear();
  stStat.clear();
  scStat.clear();
  
 }
