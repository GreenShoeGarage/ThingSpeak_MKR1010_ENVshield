View this project on [CADLAB.io](https://cadlab.io/project/23807). 

# Workspace Health Monitoring Gadget

[https://greenshoegarage.com/2020/10/learning-from-covid-19/](https://greenshoegarage.com/2020/10/learning-from-covid-19/)

## Goal

An open-source device that can monitor 8+ key environmental factors that affect people's perception of a healthy and productive workspace:

* Temperature
* Pressure
* eCO2
* Humidity
* Brightness
* Noise
* TVOC
* PM 2.5um

### Code and hardware is a prototype, no guarentee of any sort is implied. Use at your own risk.

## Functionality

1. Monitor 8 environmental factors
   1. Central processing of all sensor data
2. Record the telemetry locally
   1. 24-hour rolling log
   2. Store on microSD card
3. Display the telemetry locally
   1. Display new data point every 3 seconds
4. Transmit telemetry to web-based cloud service
   1. Send latest sensor readings every 10 minutes

## Bill of Materials

| Supplier | Product ID | Quantity | Description | Unit Price | Function | 
| :--- | :--- | :--- | :--- | :--- | :--- |  
| Mouser | 782-ABX00023 | 1 | MKR1010 | $32.10 | Processing and Comms | 
| Mouser | 782-ASX00011 | 1 | MKR ENV Shield | $34.40 | Temp, UVA/UVB/UV Index, Humidity, Pressure, Lux |
| Adafruit | 3709 | 1 | SGP30 Air Quality Sensor Breakout | $19.95 | VOC, eCO2 | 
| Adafruit | 4632 | 1 | PMSA003I Air Quality Breakout | $44.95 | Particulate Matter | 
| Adafruit | 421 | 1 | I2S MEMS Microphone Breakout SPH0645LM4H | $6.95 | Noise | 

Get the PCB from OSHPark: [https://oshpark.com/shared\_projects/NU4I8Xzd](https://oshpark.com/shared_projects/NU4I8Xzd)

PartsBox Project BOM: [https://partsbox.com/mbparks/project/1s51nh9teghyx9e4by3gwn3nxp/](https://partsbox.com/mbparks/project/1s51nh9teghyx9e4by3gwn3nxp/)

