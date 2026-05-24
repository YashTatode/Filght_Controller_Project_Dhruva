#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include "SD.h"
#include "SPI.h"
#include "FS.h"

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

const int interruptPin = 2;
const int ejectionPinBody1 = 26;
const int ejectionPinBody2 = 27;
const int ejectionPinSatellite1 = 32;
const int ejectionPinSatellite2 = 33;
const int buzzerPin = 4;
const int ledPin = 15;

int current = 0;
int pre = 0;

const int SENSOR_DATA_SIZE = 10;
float sensorData[SENSOR_DATA_SIZE];
float Pnot = 0;
int i = 0;

volatile bool interruptTriggered = false;
volatile bool firsttrigger = true;
unsigned long timerStartMillis=0;
unsigned long timerStartMillisBody=0;
unsigned long timerStartMillisSatellite=0;
unsigned long timerStartMillisSatellite1=0;
const unsigned long timerDurationBody = 16000;

File FunctionFile;
File MainDataFile;
File DataFile;

void IRAM_ATTR handleInterrupt() {
  interruptTriggered = true;
}

void setup(void) {
  Serial.begin(115200);

  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(ejectionPinBody1, OUTPUT);
  pinMode(ejectionPinBody2, OUTPUT);
  pinMode(ejectionPinSatellite1, OUTPUT);
  pinMode(ejectionPinSatellite2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  digitalWrite(ejectionPinBody1, LOW);
  digitalWrite(ejectionPinBody2, LOW);
  digitalWrite(ejectionPinSatellite1, LOW);
  digitalWrite(ejectionPinSatellite2, LOW);


  initialize();
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    
    mpuded();
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");

  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW); 
  delay(1000);

  Serial.println(F("BMP280 test"));
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

  // status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    
    bmpded();//

    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
  }
 
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Pnot = bmp.readPressure() / 100;

  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW); 
  delay(1000);

  if (!SD.begin()) {
    Serial.println("Card initialization failed!");
    
    sdded();
  }

  MainDataFile = SD.open("/MainData.txt", FILE_APPEND);
  DataFile=SD.open("/Data.txt", FILE_APPEND);
  FunctionFile = SD.open("/Function.txt", FILE_APPEND);

  if (!MainDataFile || !DataFile || !FunctionFile) {
    Serial.println("Error opening files!");
    sdded();
  }

  FunctionFile.println("Function started:" + String(millis()));
  DataFile.println("Data Started:");
  MainDataFile.println("Function started:");

  FunctionFile.close();
  DataFile.close();
  MainDataFile.close();

  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW); 
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);

    delay(100);
}

void loop() {

  if (interruptTriggered) {
    if (digitalRead(interruptPin) == LOW){
      timerStartMillis = millis();
      interruptTriggered = false;
      FunctionEvent("Interrupt trigger at: ", i, sensorData);
    }
    else{
      timerStartMillis = 0;
      timerStartMillisBody = 0;
      timerStartMillisSatellite = 0;
      timerStartMillisSatellite1 = 0;
      interruptTriggered = false;
      firsttrigger = true;
    }
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  sensorData[0] = a.acceleration.x;
  sensorData[1] = a.acceleration.y;
  sensorData[2] = a.acceleration.z;
  sensorData[3] = g.gyro.x;
  sensorData[4] = g.gyro.y;
  sensorData[5] = g.gyro.z;
  sensorData[6] = temp.temperature;
  sensorData[7] = bmp.readTemperature();
  sensorData[8] = bmp.readPressure() / 100;
  sensorData[9] = bmp.readAltitude(Pnot);
  current = sensorData[9];
  Serial.println(current);
  i = i + 1;

  SerialDisplay(i, sensorData);

  if (sensorData[9] >= 1000.0 &&
      ((abs(sensorData[0]) >= 8.0 && abs(sensorData[0]) <= 10.0) ||
       (abs(sensorData[2]) >= 8.0 && abs(sensorData[2]) <= 10.0)) && firsttrigger) {
      digitalWrite(ejectionPinBody1, HIGH);
      FunctionEvent("Ejection charge 1 of Body turned on due to altitude and Gyro at: ", i, sensorData);
      timerStartMillis=0;
      timerStartMillisBody= millis();
      firsttrigger = false;
    }

    if(current>pre){
      pre = current;
    }

    if (current-pre<-100 && firsttrigger){
      digitalWrite(ejectionPinBody1, HIGH);
      FunctionEvent("Ejection charge 1 of Body turned on due to Backup altitude at: ", i, sensorData);
      timerStartMillis=0;
      timerStartMillisBody= millis();
      firsttrigger = false;
    }

    if (timerStartMillis!=0 && millis() - timerStartMillis >= timerDurationBody && firsttrigger) {
        Serial.println("deployed");
        digitalWrite(ejectionPinBody1, HIGH);
        FunctionEvent("Ejection charge 1 of Body turned on due to timer at: ", i, sensorData);
        timerStartMillis=0;
        timerStartMillisBody= millis();
        firsttrigger = false;
    }

    if (timerStartMillisBody!=0 && millis() - timerStartMillisBody >= 2000) {
      digitalWrite(ejectionPinBody1, LOW);
      digitalWrite(ejectionPinBody2, HIGH);
      FunctionEvent("Ejection charge 2 of Body turned on after 2 second of 1st ejection charge of body at: ", i, sensorData);
      timerStartMillisBody=0;
      timerStartMillisSatellite= millis();
    }
    if (timerStartMillisSatellite!=0 && millis() - timerStartMillisSatellite >= 2000) {
      digitalWrite(ejectionPinBody2, LOW);
      digitalWrite(ejectionPinSatellite1, HIGH);
      FunctionEvent("Ejection charge 1 of Satellite turned on after 2 second of 2nd ejection charge of body at: ", i, sensorData);
      timerStartMillisSatellite=0;
      timerStartMillisSatellite1= millis();
    }
    if (timerStartMillisSatellite1!=0 && millis() - timerStartMillisSatellite1 >= 2000) {
      digitalWrite(ejectionPinSatellite1, LOW);
      digitalWrite(ejectionPinSatellite2, HIGH);
      FunctionEvent("Ejection charge 2 of Satellite turned on after 2 second of 1st ejection charge of satellite at: ", i, sensorData);
      timerStartMillisSatellite1=0;
    }

MainData(i, sensorData);
delay(100);
}

void MainData(int i, float sensorData[]) {
  MainDataFile = SD.open("/MainData.txt", FILE_APPEND);
  DataFile=SD.open("/Data.txt", FILE_APPEND);

  if (MainDataFile) {
    MainDataFile.print(i);
    MainDataFile.print(millis());
    for (int index = 0; index < SENSOR_DATA_SIZE; ++index) {
      MainDataFile.print(" , ");
      MainDataFile.print(sensorData[index]);
      if ((i % 20) >= 1 && (i % 20) <= 10) {
        digitalWrite(ledPin, HIGH); // Turn LED ON
      } else {
        digitalWrite(ledPin, LOW);  // Turn LED OFF
      }
    }
    MainDataFile.println();
    MainDataFile.close();
    Serial.println("Saved Data");
  } else {
    Serial.println("Error opening Data file!");
  }
  if (DataFile) {
    DataFile.println(i);
    DataFile.print("Altitude = ");
    DataFile.print(sensorData[9]);
    DataFile.println(" m");

    DataFile.print("Acceleration X: ");
    DataFile.print(sensorData[0]);
    DataFile.print(", Y: ");
    DataFile.print(sensorData[1]);
    DataFile.print(", Z: ");
    DataFile.print(sensorData[2]);
    DataFile.println(" m/s^2");

    DataFile.print("Rotation X: ");
    DataFile.print(sensorData[3]);
    DataFile.print(", Y: ");
    DataFile.print(sensorData[4]);
    DataFile.print(", Z: ");
    DataFile.print(sensorData[5]);
    DataFile.println(" rad/s");

    DataFile.print("Temperature MPU: ");
    DataFile.print(sensorData[6]);
    DataFile.println(" *C");

    DataFile.print("Temperature BMP: ");
    DataFile.print(sensorData[7]);
    DataFile.println(" *C");
    DataFile.print("Pressure = ");
    DataFile.print(sensorData[8]);
    DataFile.println(" hPa");

    DataFile.println("");
    DataFile.close();
  } else {
    Serial.println("Error opening Function file!");
  }
}


void FunctionEvent(const char* message, int i, float sensorData[]) {
  FunctionFile = SD.open("/Function.txt", FILE_APPEND);

  if (FunctionFile) {
    FunctionFile.print(message);
    FunctionFile.println(i);
    FunctionFile.println(millis());
    FunctionFile.print("Altitude = ");
    FunctionFile.print(sensorData[9]);
    FunctionFile.println(" m");

    FunctionFile.print("Acceleration X: ");
    FunctionFile.print(sensorData[0]);
    FunctionFile.print(", Y: ");
    FunctionFile.print(sensorData[1]);
    FunctionFile.print(", Z: ");
    FunctionFile.print(sensorData[2]);
    FunctionFile.println(" m/s^2");

    FunctionFile.print("Rotation X: ");
    FunctionFile.print(sensorData[3]);
    FunctionFile.print(", Y: ");
    FunctionFile.print(sensorData[4]);
    FunctionFile.print(", Z: ");
    FunctionFile.print(sensorData[5]);
    FunctionFile.println(" rad/s");

    FunctionFile.print("Temperature MPU: ");
    FunctionFile.print(sensorData[6]);
    FunctionFile.println(" *C");

    FunctionFile.print("Temperature BMP: ");
    FunctionFile.print(sensorData[7]);
    FunctionFile.println(" *C");
    FunctionFile.print("Pressure = ");
    FunctionFile.print(sensorData[8]);
    FunctionFile.println(" hPa");

    FunctionFile.println("");

    FunctionFile.close();
  } else {
    Serial.println("Error opening Function file!");
  }
}

void SerialDisplay(int i, float sensorData[]){
  Serial.println(i);
  
  Serial.print("Altitude = ");
  Serial.print(sensorData[9]);
  Serial.println(" m");

  Serial.print("Acceleration X: ");
  Serial.print(sensorData[0]);
  Serial.print(", Y: ");
  Serial.print(sensorData[1]);
  Serial.print(", Z: ");
  Serial.print(sensorData[2]);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(sensorData[3]);
  Serial.print(", Y: ");
  Serial.print(sensorData[4]);
  Serial.print(", Z: ");
  Serial.print(sensorData[5]);
  Serial.println(" rad/s");

  Serial.print("Temperature MPU: ");
  Serial.print(sensorData[6]);
  Serial.println(" degC");

  Serial.print("Temperature BMP= ");
  Serial.print(sensorData[7]);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(sensorData[8]);
  Serial.println(" hPa");

  Serial.println("");
}

void initialize()
{
   digitalWrite(buzzerPin, HIGH); 
  delay(200);                    
  digitalWrite(buzzerPin, LOW);  
  delay(200);
  digitalWrite(buzzerPin, HIGH); 
  delay(200);                    
  digitalWrite(buzzerPin, LOW);  
}

void mpuded()
{
  digitalWrite(buzzerPin, HIGH); 
  delay(200);                    
  digitalWrite(buzzerPin, LOW);  
  delay(200);
  digitalWrite(buzzerPin, HIGH);  
  delay(200);   
  digitalWrite(buzzerPin, LOW);

}

void bmpded()
{
  digitalWrite(buzzerPin, HIGH);  
  delay(200);                     
  digitalWrite(buzzerPin, LOW);   
  delay(200);
  digitalWrite(buzzerPin, HIGH);  
  delay(200);   
  digitalWrite(buzzerPin, LOW);

}

void sdded()
{
  digitalWrite(buzzerPin, HIGH);  
  delay(200);                      
  digitalWrite(buzzerPin, LOW);  
  delay(200);
  digitalWrite(buzzerPin, HIGH);  
  delay(200);   
  digitalWrite(buzzerPin, LOW);
  delay(200);   
  digitalWrite(buzzerPin, HIGH);  
  delay(200);   
  digitalWrite(buzzerPin, LOW);
}