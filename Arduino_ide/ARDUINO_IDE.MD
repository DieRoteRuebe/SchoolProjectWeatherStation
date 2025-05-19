# Weather Station ESP32 Project

This project is a smart IoT-based weather station using an ESP32 microcontroller. It connects to Wi-Fi, reads temperature, humidity, and pressure data from a BME280 sensor, displays it on an OLED screen, and sends the data to an MQTT broker in JSON format. It then enters deep sleep mode for power efficiency and repeats the process at fixed intervals.

Features

* Collects temperature, humidity, and pressure data using the BME280 sensor
* Displays the data on a 128x64 OLED display (SSD1306)
* Synchronizes time using NTP servers
* Publishes JSON data to an MQTT topic
* Uses deep sleep to conserve power (5-minute sleep interval)
* Fully implemented in Arduino C++

# Hardware Components

* ESP32 Dev Board
* BME280 Sensor (I2C interface)
* 128x64 OLED Display (SSD1306, I2C)
* Jumper wires, breadboard (optional for prototyping)

# Wiring

| Component  | ESP32 Pin |
| ---------- | --------- |
| BME280 SDA | GPIO 4    |
| BME280 SCL | GPIO 32   |
| OLED SDA   | GPIO 4    |
| OLED SCL   | GPIO 32   |

# Libraries Used

* WiFi.h — for Wi-Fi connectivity
* Wire.h — I2C communication
* Adafruit\_Sensor.h, Adafruit\_BME280.h — sensor library
* Adafruit\_GFX.h, Adafruit\_SSD1306.h — OLED graphics
* PubSubClient.h — MQTT communication
* time.h — time synchronization with NTP
* ArduinoJson.h — JSON encoding of sensor data

# MQTT Configuration

* Broker IP: 192.168.108.14
* Port: 1883
* Topic: Wuerzburg2/KOS/A105/data

# Metadata Configuration

* Device ID: esp2
* Location: Mariehamn, Åland (locationShort: "Mariehamn")

# Sleep Interval

The ESP32 enters deep sleep for 5 minutes after each successful sensor read and MQTT publish. This helps reduce power consumption.

# How It Works

1. Initializes I2C, Display, Wi-Fi, and MQTT
2. Connects to Wi-Fi and synchronizes time via NTP
3. Initializes BME280 sensor and reads data
4. Displays the sensor data and timestamp on OLED
5. Publishes the data as a JSON object to MQTT broker
6. Sleeps for 5 minutes before restarting

# Project Usage

* Flash the code to your ESP32 using Arduino IDE or PlatformIO
* Connect the hardware as described
* Ensure your MQTT broker is running
* Open the Serial Monitor at 115200 baud to see logs

# Sample MQTT JSON Payload

{
"location": "Mariehamn",
"temp": 22.45,
"hum": 48.72,
"pressure": 1013.56,
"time": "2025-05-19 14:30:00",
"esp\_id": "esp2"
}

# Notes

* Make sure your Wi-Fi credentials and broker IP are correctly set in the code
* The BME280 sensor uses I2C address 0x76 by default
* Uses retries and reconnection logic for stable MQTT and Wi-Fi operation

# License

for licensing please view the top directory and the including LICENSE.MD and THIRDPARTY.MD

