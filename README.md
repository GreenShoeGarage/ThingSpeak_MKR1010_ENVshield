### Workspace Health Monitoring Gadget

## Goal
An open-source device that can monitor 6 key environmental factors that affect people's perception of a healthy and productive workspace:

- Temperature
- Pressure
- eCO2
- Humidity
- Brightness
- Noise
- TVOC
- PM 2.5um

Measure 8+ factors that affect the environmental quality of a room.... temp, humidity, atmospheric pressure, lux, eCO2, VOC, particulate matter, sound/noise level. Pack it up and send it to a website for remote monitoring. Wanted something a bit more rugged than a breadboard as I start testing it. Next phase is learning from real world environment and lots of code cleanup... 'clooged' together a lot of example code, now I want to write something more usable and maintainable.

## Functionality

1. Monitor multiple environmental factors
    1. Central processing of all sensor data
2. Record the telemetry locally
    1. 24-hour rolling log
3. Display the telemetry locally
    1. Display new data point every 3 seconds
4. Transmit telemetry to web-based cloud service
    

## Bill of Materials
Supplier | Product ID | Quantity Description | Unit Price | Function
--- | --- | --- | --- | ---
Mouser | 782-ABX00023 | 1 | MKR1010 | $32.10 | Processing and Comms	
Mouser | 782-ASX00011 | 1 | MKR ENV Shield | $34.40 | Temp | UVA/UVB/UV Index, Humidity, Pressure, Lux	
Adafruit | 3709 |1 | SGP30 Air Quality Sensor Breakout | $19.95 | VOC, eCO2	
Adafruit | 4632 | 1 |PMSA003I Air Quality Breakout | $44.95 | Particulate Matter
Adafruit | 421|  1 | I2S MEMS Microphone Breakout SPH0645LM4H | $6.95 | Noise

Get the PCB from OSHPark:
https://oshpark.com/shared_projects/NU4I8Xzd
