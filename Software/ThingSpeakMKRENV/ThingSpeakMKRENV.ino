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

  Microphone Connections
    GND connected GND
    3.3V connected 3.3V (Feather, Zero) or VCC (MKR1000, MKRZero)
    LRCLK / WS connected to pin 0 (Feather, Zero) or pin 3 (MKR1000, MKRZero)
    BCLK connected to pin 1 (Feather, Zero) or pin 2 (MKR1000, MKRZero)
    DOUT connected to pin 9 (Zero) or pin A6 (MKR1000, MKRZero
    SEL N/C

*/
#include <SPI.h>
#include <SD.h>
#include "ThingSpeak.h"
#include <WiFiNINA.h>
#include <Arduino_MKRENV.h>
#include "secrets.h"
#include <I2S.h>
#include <ArduinoLowPower.h>

#define SAMPLES 128
#define CSPIN 4

float temperature = 0.0;
float humidity    = 0.0;
float pressure    = 0.0;
float illuminance = 0.0;
float uva         = 0.0;
float uvb         = 0.0;
float uvIndex     = 0.0;
float noiseLevel   = 0.0;

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
    ;
  }

  Serial.print("Initializing MKR ENV shield...");
  if (!ENV.begin()) {
    Serial.println("FAILED to initialize MKR ENV shield!");
    while (true);
  }
  Serial.println("MKR ENV shield successfully initialized");


  Serial.print("Initializing I2S microphone...");
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
    Serial.println("FAILED to initialize I2S microphone!");
    while (true); // do nothing
  }
  Serial.println("I2S microphone successfully initialized");



  Serial.print("Initializing WiFi module...");
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  Serial.print("Wifi Firmware Version: ");
  Serial.println(fv);
  Serial.println("WiFi successfully initialized");


  Serial.print("Initializing SD card...");
  if (!SD.begin(CSPIN))
  {
    Serial.println("initialization failed!");
    while (true);
  }
  Serial.println("SD card successfully initialized.");

  ThingSpeak.begin(client);  //Initialize ThingSpeak
}




void loop() {
  readSensors();
  printTelemetry();
  sendToThingspeak();
  writeTelemetrySD();
  delay(2000);
  //LowPower.sleep(3000);
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
}





void printTelemetry() {
  Serial.println(millis());
  Serial.print("temperature: ");
  Serial.println(temperature);
  Serial.print("humidity: ");
  Serial.println(humidity);
  Serial.print("pressure: ");
  Serial.println(pressure);
  Serial.print("illuminance: ");
  Serial.println(illuminance);
  Serial.print("uva: ");
  Serial.println(uva);
  Serial.print("uvb: ");
  Serial.println(uvb);
  Serial.print("uvIndex: ");
  Serial.println(uvIndex);
  Serial.print("Noise Level: ");
  Serial.println(noiseLevel);
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
    ThingSpeak.setField(5, uva);
    ThingSpeak.setField(6, uvb);
    ThingSpeak.setField(7, uvIndex);
    ThingSpeak.setField(8, noiseLevel);

    // write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200) {
      Serial.println("Channel update successful.");
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
      myFile.close();

      Serial.println("Telemetry written to SD card.");
      lastSDWriteTime = millis();
    }
    else {
      Serial.print("WARNING! Error opening file: ");
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
