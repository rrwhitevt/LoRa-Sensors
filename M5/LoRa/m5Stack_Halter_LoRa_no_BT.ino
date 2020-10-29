/*
  please add TinyGPSPlus to your library first........
  TinyGPSPlus file in M5stack lib examples -> modules -> GPS -> TinyGPSPlus-1.0.2.zip
*/

#include <M5Stack.h>
#include <TinyGPS++.h>
#include <M5LoRa.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

static const uint32_t GPSBaud = 9600;
String sensorname = "7";
int device_id = 1007;

// Set up objects
TinyGPSPlus gps;
MPU9250 IMU;

// The serial connection to the GPS device
HardwareSerial ss(2);


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

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}


void setup()
{
  M5.begin();
  M5.Lcd.setTextFont(1);
  M5.Lcd.setCursor(0, 0, 1);
  
  ss.begin(GPSBaud);
  Serial.println("GPS Initialized");
  M5.Lcd.println("GPS Initialized");
  
  Wire.begin();
  IMU.initMPU9250();
  Serial.println("IMU Initialized");
  M5.Lcd.println("IMU Initialized");
  
  SD.begin();
  Serial.println("SD Card Initialized");
  M5.Lcd.println("SD Card Initialized");
  

  //Open the SD datalog file
  File f = SD.open("/datalog.txt");
  if(!f) { 
    Serial.println("File does not exist");
    Serial.println("Creating file...");
    writeFile(SD, "/datalog.txt", "");
  } else {
    Serial.println("File exists, will append data");
  }
  f.close();
  delay(2000);

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(); // default set CS, reset, IRQ pin
  Serial.println("LoRa Sender");
  M5.Lcd.println("LoRa Sender");

  // frequency in Hz (433E6, 866E6, 915E6)
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    M5.Lcd.println("Starting LoRa failed!");
    while (1);
  }

  // LoRa.setSyncWord(0x69);
  Serial.println("LoRa init succeeded.");
  M5.Lcd.println("LoRa init succeeded.");
  delay(2000);
  
  Serial.begin(115200);
}

void loop()
{
  M5.Lcd.fillScreen(TFT_BLACK);


  //GPS Values for SD card/BT
  String nSat = String(gps.satellites.value(), 5);
  String Dop = String(gps.hdop.value(), 5);
  String Lat = String(gps.location.lat(), 6);
  String Long = String(gps.location.lng(), 6);
  String DateM = String(gps.date.month());
  String DateD = String(gps.date.day());
  String DateY = String(gps.date.year());
  String TimeH = String(gps.time.hour());
  String TimeM = String(gps.time.minute());
  String TimeS = String(gps.time.second());
  String Alt = String(gps.altitude.meters(),2);
  String Vol = String(gps.speed.kmph(), 2);

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
  IMU.magbias[0] = +470.;
  IMU.magbias[1] = +120.;
  IMU.magbias[2] = +125.;

  IMU.mx = (float)IMU.magCount[0]*IMU.mRes*IMU.magCalibration[0] - IMU.magbias[0];
  IMU.my = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1] - IMU.magbias[1];
  IMU.mz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2] - IMU.magbias[2];

  //Calculate quaternion
  IMU.updateTime();
  MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx*DEG_TO_RAD,
                         IMU.gy*DEG_TO_RAD, IMU.gz*DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);
                         
  // Print acceleration values in milligs!
  String ax = String(1000*IMU.ax);
  String ay = String(1000*IMU.ay);
  String az = String(1000*IMU.az);

  // Print gyro values in degree/sec
  String gx = String(IMU.gx, 3);
  String gy = String(IMU.gy, 3);
  String gz = String(IMU.gz, 3);

  // Print mag values in degree/sec
  String mx = String(IMU.mx);
  String my = String(IMU.my);
  String mz = String(IMU.mz);

  IMU.tempCount = IMU.readTempData();  // Read the adc values
  // Temperature in degrees Centigrade
  IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;

  //Calculate pitch yaw and roll
  IMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
  IMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() * *(getQ()+2)));
  IMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
  IMU.pitch *= RAD_TO_DEG;
  IMU.yaw   *= RAD_TO_DEG;
  IMU.yaw   -= 8.5;
  IMU.roll  *= RAD_TO_DEG;

  String oyaw = String(IMU.yaw);
  String opitch = String(IMU.pitch);
  String oroll = String(IMU.roll);
  String tm = String(IMU.temperature);
  String ms = String(millis());

  //Create Outstring to append to SD card
  String GPSOut = sensorname+","+Lat+","+Long+","+Vol+","+tm+","+ax+","+ay+","+az+","+gx+","+gy+","+gz+","+mx+","+my+","+mz;
  String IMUOut = tm+","+ax+","+ay+","+az+","+gx+","+gy+","+gz+","+mx+","+my+","+mz;
  String outString = ms+","+DateM+","+DateD+","+DateY+","+TimeH+","+TimeM+","+TimeS+","+nSat+","+Dop+","+Lat+","+Long+","+Alt+","+Vol+tm+","+ax+","+ay+","+az+","+gx+","+gy+","+gz+","+mx+","+my+","+mz+","+oyaw+","+opitch+","+oroll;
  Serial.println(outString);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setCursor(0, 0, 1);
  M5.Lcd.println(outString);

  //Send packet out through LoRa
  LoRa.setTxPower(10);
  LoRa.beginPacket();
  LoRa.print("<");
  LoRa.print(device_id);
  LoRa.print(">field1=");
  LoRa.print(GPSOut);
  LoRa.endPacket();

  //print lora string
  Serial.print("<");
  Serial.print(device_id);
  Serial.print(">field1=");
  Serial.println(GPSOut);
  
  //appendFile(SD, "/datalog.txt", outString.c_str());

  smartDelay(1000);

 }
