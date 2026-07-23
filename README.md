# karakterisasi-dinamika-plant
Source code karakterisasi dinamika plant hidroponik NFT menggunakan ESP32
# Karakterisasi Dinamika Plant Hidroponik NFT

Repository ini berisi source code ESP32 yang digunakan untuk karakterisasi dinamika plant pada sistem hidroponik NFT.

## Hardware
- ESP32
- ADS1115
- Sensor pH DFRobot
- Sensor EC DFRobot
- Sensor DO DFRobot
- Sensor suhu DS18B20

## Library
- WiFi
- PubSubClient
- Adafruit ADS1X15
- OneWire
- DallasTemperature
- DFRobot_PH_ADS
- DFRobot_EC_ADS
- DFRobot_DO_ADS

## Output
Data dikirim melalui MQTT dalam format JSON yang berisi:
- Suhu
- pH
- Tegangan pH (mV)
- EC
- Tegangan EC (mV)
- DO
- Tegangan DO (mV)
