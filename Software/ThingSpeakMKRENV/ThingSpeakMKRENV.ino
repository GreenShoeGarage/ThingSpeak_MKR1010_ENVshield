/*
  WriteMultipleFields

  Description: Writes values to fields 1,2,3,4 and status in a single ThingSpeak update every 20 seconds.

  Hardware: Arduino MKR WiFi 1010

  !!! IMPORTANT - Modify the secrets.h file for this project with your network connection and ThingSpeak channel details. !!!

  Note:
  - Requires WiFiNINA library
  - This example is written for a network using WPA encryption. For WEP or WPA, change the WiFi.begin() call accordingly.

  ThingSpeak ( https://www.thingspeak.com ) is an analytic IoT platform service that allows you to aggregate, visualize, and
  analyze live data streams in the cloud. Visit https://www.thingspeak.com to sign up for a free account and create a channel.

  Documentation for the ThingSpeak Communication Library for Arduino is in the README.md folder where the library was installed.
  See https://www.mathworks.com/help/thingspeak/index.html for the full ThingSpeak documentation.

  For licensing information, see the accompanying license file.

  Copyright 2018, The MathWorks, Inc.

  Microphone Connections (Adafruit I2S MEMS Microphone Breakout SPH0645LM4H)
    GND connected GND
    3.3V connected 3.3V (Feather, Zero) or VCC (MKR1000, MKRZero)
    LRCLK / WS connected to pin 0 (Feather, Zero) or pin 3 (MKR1000, MKRZero)
    BCLK connected to pin 1 (Feather, Zero) or pin 2 (MKR1000, MKRZero)
    DOUT connected to pin 9 (Zero) or pin A6 (MKR1000, MKRZero
    SEL N/C

  Adafruit PM2.5 sensor (MSA003I Air Quality Breakout (PM0.3-100um))
    I2C address 0x12 (cannot be changed)
    3.3V
    SDA, SCL, 3V, GNF

  SGP30 Air Quality Sensor Breakout (TVOC and eCO2)
    The sensor uses I2C address 0x58
    Since the sensor chip uses 3 VDC for logic, we have included a voltage regulator on board that will take 3-5VDC and safely convert it down
    SDA, SCL, 3.3V or 5V, GND

test
*/
#include <SPI.h>
#include <SD.h>
#include "ThingSpeak.h"
#include <WiFiNINA.h>
#include <Arduino_MKRENV.h>
#include "secrets.h"
#include <I2S.h>
#include <ArduinoLowPower.h>
#include "Adafruit_PM25AQI.h"
#include <utility/wifi_drv.h>
#include "Adafruit_SGP30.h"



#define SAMPLES 128
#define CSPIN 4
#define GREEN_LED 25
#define RED_LED 26
#define BLUE_LED 27


float temperature = 0.0;
float humidity    = 0.0;
float pressure    = 0.0;
float illuminance = 0.0;
float uva         = 0.0;
float uvb         = 0.0;
float uvIndex     = 0.0;
float noiseLevel   = 0.0;
int partcount_03um = 0;
int partcount_05um = 0;
int partcount_10um = 0;
int partcount_25um = 0;
int partcount_50um = 0;
int partcount_100um = 0;
int sensorReading_tvoc = 0;
int sensorReading_eco2 = 0;

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
PM25_AQI_Data aqiData;

Adafruit_SGP30 sgp;

unsigned long lastSDWriteTime = 0;
unsigned long SDwriteDelay = 60000;
unsigned long lastTransmitTime = 0;
unsigned long transmitDelay = 60000;

char ssid[] = SECRET_SSID;   // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password
WiFiClient  client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

const char* FILENAME = "telemetr.csv";
File myFile;

void setup() {
  Serial.begin(115200);  // Initialize serial
  while (!Serial) {
    blinkLedError();
  }

  Serial.print("Initializing MKR ENV shield...");
  if (!ENV.begin()) {
    Serial.println("FAILED to initialize MKR ENV shield!");
    while (true) {
      blinkLedError();
    }
  }
  Serial.println("MKR ENV shield successfully initialized");


  Serial.print("Initializing I2S microphone...");
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
    Serial.println("FAILED to initialize I2S microphone!");
    while (true) {
      blinkLedError();
    }
  }
  Serial.println("I2S microphone successfully initialized");


  Serial.print("Initializing PM 2.5 AQI Sensor...");
  if (! aqi.begin_I2C()) {      // connect to the sensor over I2C
    Serial.println("Could not find PM 2.5 sensor!");
    while (true) {
      blinkLedError();
    }
  }
  Serial.println("PM 2.5 AQI sensor successfully initialized!");


  Serial.print("Initializing SGP30 sensor...");
  if (! sgp.begin()) {
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("SGP30 sensor successfully initialized!");


  Serial.print("Initializing WiFi module...");
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true) {
      blinkLedError();
    }
  }
  String fv = WiFi.firmwareVersion();
  Serial.print("Wifi Firmware Version: ");
  Serial.println(fv);
  Serial.println("WiFi successfully initialized");


  Serial.print("Initializing SD card...");
  if (!SD.begin(CSPIN))
  {
    Serial.println("initialization failed!");
    while (true) {
      blinkLedError();
    }
  }
  Serial.println("SD card successfully initialized.");

  ThingSpeak.begin(client);  //Initialize ThingSpeak

  WiFiDrv::pinMode(GREEN_LED, OUTPUT);  //GREEN
  WiFiDrv::pinMode(RED_LED, OUTPUT);  //RED
  WiFiDrv::pinMode(BLUE_LED, OUTPUT);  //BLUE
  WiFiDrv::digitalWrite(GREEN_LED, HIGH); // for full brightness
  WiFiDrv::digitalWrite(RED_LED, LOW); // for full brightness
  WiFiDrv::digitalWrite(BLUE_LED, LOW); // for full brightness
}




void loop() {
  readSensors();
  printTelemetry();
  sendToThingspeak();
  writeTelemetrySD();
  delay(2000);
  //LowPower.idle(3000);
}




void readSensors() {
  temperature = ENV.readTemperature();
  humidity    = ENV.readHumidity();
  pressure    = ENV.readPressure();
  illuminance = ENV.readIlluminance();
  uva         = ENV.readUVA();
  uvb         = ENV.readUVB();
  uvIndex     = ENV.readUVIndex();
  noiseLevel = getNoiseReading();
  readAQI();
  partcount_03um = aqiData.particles_03um;
  partcount_05um = aqiData.particles_05um;
  partcount_10um = aqiData.particles_10um;
  partcount_25um = aqiData.particles_25um;
  partcount_50um = aqiData.particles_50um;
  partcount_100um = aqiData.particles_100um;
  sensorReading_tvoc = sgp.TVOC;
  sensorReading_eco2 = sgp.eCO2;
}





void printTelemetry() {
  Serial.println(F("------------------------------------------------------------"));
  Serial.print(F("Time (ms) since last reboot: "));
  Serial.println(millis());
  Serial.println();

  Serial.print(F(">> Environmental Conditions: "));
  Serial.println();
  Serial.print(F("Temperature (C): "));
  Serial.println(temperature);
  Serial.print(F("Humidity (%): "));
  Serial.println(humidity);
  Serial.print(F("Atmosphere Pressure (kPa): "));
  Serial.println(pressure);
  Serial.print(F("Illuminance (Lux): "));
  Serial.println(illuminance);
  Serial.print(F("UVA: "));
  Serial.println(uva);
  Serial.print(F("UVB: "));
  Serial.println(uvb);
  Serial.print(F("UV Index: "));
  Serial.println(uvIndex);
  Serial.println();

  Serial.print(F(">> Noise Level: "));
  Serial.println(noiseLevel);
  Serial.println();

  Serial.println(F(">> Air Quality: Particulate Matter"));
  Serial.println(F("Concentration Units (standard)"));
  Serial.print(F("PM 1.0: ")); Serial.print(aqiData.pm10_standard);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(aqiData.pm25_standard);
  Serial.print(F("\t\tPM 10: ")); Serial.println(aqiData.pm100_standard);
  Serial.println(F("-----"));
  Serial.println(F("Concentration Units (environmental)"));
  Serial.print(F("PM 1.0: ")); Serial.print(aqiData.pm10_env);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(aqiData.pm25_env);
  Serial.print(F("\t\tPM 10: ")); Serial.println(aqiData.pm100_env);
  Serial.println(F("-----"));
  Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(partcount_03um);
  Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(partcount_05um);
  Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(partcount_10um);
  Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(partcount_25um);
  Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(partcount_50um);
  Serial.print(F("Particles > 50 um / 0.1L air:")); Serial.println(partcount_100um);
  Serial.println();

  Serial.println(F(">> Air Quality: eCO2 and TVOC"));
  Serial.print(F("eCO2: "));
  Serial.println(sensorReading_eco2);
  Serial.print(F("TVOC: "));
  Serial.println(sensorReading_tvoc);
  Serial.println();
  Serial.println(F("------------------------------------------------------------"));

  for (int x = 0; x < 3; x++) {
    Serial.println();
  }

}




void sendToThingspeak() {
  // Connect or reconnect to WiFi
  if ((millis() - lastTransmitTime) > transmitDelay) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(SECRET_SSID);
      while (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
        Serial.print(".");
        delay(5000);
      }
      Serial.println("\nConnected.");
    }

    // set the fields with the values
    ThingSpeak.setField(1, temperature);
    ThingSpeak.setField(2, humidity);
    ThingSpeak.setField(3, pressure);
    ThingSpeak.setField(4, illuminance);
    ThingSpeak.setField(5, uvIndex);
    ThingSpeak.setField(6, noiseLevel);
    ThingSpeak.setField(7, partcount_25um);
    ThingSpeak.setField(8, sensorReading_eco2);

    // write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200) {
      Serial.println("Telemetry upload to ThingSpeak successful.");
    }
    else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    lastTransmitTime = millis();
  }
}




void writeTelemetrySD() {
  noInterrupts();
  if ((millis() - lastSDWriteTime) > SDwriteDelay) {
    myFile = SD.open(FILENAME, FILE_WRITE);
    if (myFile)
    {
      myFile.print(temperature);
      myFile.print(",");
      myFile.print(humidity);
      myFile.print(",");
      myFile.print(pressure);
      myFile.print(",");
      myFile.print(illuminance);
      myFile.print(",");
      myFile.print(uva);
      myFile.print(",");
      myFile.print(uvb);
      myFile.print(",");
      myFile.print(uvIndex);
      myFile.print(",");
      myFile.println(noiseLevel);
      myFile.print(",");
      myFile.println(partcount_03um);
      myFile.print(",");
      myFile.println(partcount_05um);
      myFile.print(",");
      myFile.println(partcount_10um);
      myFile.print(",");
      myFile.println(partcount_25um);
      myFile.print(",");
      myFile.println(partcount_50um);
      myFile.print(",");
      myFile.println(partcount_100um);
      myFile.print(",");
      myFile.println(sensorReading_eco2);
      myFile.print(",");
      myFile.println(sensorReading_tvoc);
      myFile.close();

      Serial.println("Telemetry write to SD card is successful.");
      lastSDWriteTime = millis();
    }
    else {
      Serial.print("WARNING! SD Card problem. Error opening file: ");
      Serial.println(FILENAME);
    }


  }
  interrupts();
}



float getNoiseReading() {
  int samples[SAMPLES];

  for (int i = 0; i < SAMPLES; i++) {
    int sample = 0;
    while ((sample == 0) || (sample == -1) ) {
      sample = I2S.read();
    }
    // convert to 18 bit signed
    sample >>= 14;
    samples[i] = sample;
  }

  // ok we hvae the samples, get the mean (avg)
  float meanval = 0;
  for (int i = 0; i < SAMPLES; i++) {
    meanval += samples[i];
  }
  meanval /= SAMPLES;

  // subtract it from all sapmles to get a 'normalized' output
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] -= meanval;
  }

  // find the 'peak to peak' max
  float maxsample, minsample;
  minsample = 100000;
  maxsample = -100000;
  for (int i = 0; i < SAMPLES; i++) {
    minsample = min(minsample, samples[i]);
    maxsample = max(maxsample, samples[i]);
  }
  float noiseLevel = maxsample - minsample;
  return noiseLevel;
}


void readAQI() {
  if (! aqi.read(&aqiData)) {
    Serial.println("Could not read from AQI");
    delay(500);  // try again in a bit!
    return;
  }
  //Serial.println("AQI reading success");
}


void blinkLedError() {
  WiFiDrv::digitalWrite(GREEN_LED, LOW); // for full brightness
  WiFiDrv::digitalWrite(RED_LED, LOW); // for full brightness
  WiFiDrv::digitalWrite(BLUE_LED, HIGH); // for full brightness
  delay(1000);
  WiFiDrv::digitalWrite(GREEN_LED, LOW); // for full brightness
  WiFiDrv::digitalWrite(RED_LED, HIGH); // for full brightness
  WiFiDrv::digitalWrite(BLUE_LED, LOW); // for full brightness
  delay(1000);
}
