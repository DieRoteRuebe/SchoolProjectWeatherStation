# ESP32 Environmental Sensor with MQTT and Display

## Overview

This project uses an ESP32 microcontroller to read environmental data from a **BME280** sensor, display the readings on an **I2C OLED display**, and publish the data to an **MQTT broker**. It also synchronizes system time via **NTP** and periodically enters **deep sleep** to conserve power.

## Features

* Reads **temperature**, **humidity**, and **pressure** from a BME280 sensor via I2C.
* Synchronizes system time using SNTP (Simple Network Time Protocol).
* Displays sensor data and metadata (time, location) on an I2C OLED display.
* Publishes formatted JSON data to an MQTT topic.
* Enters deep sleep for 1 minute between readings to save power.

## Hardware

* ESP32 microcontroller
* BME280 environmental sensor
* OLED display via I2C
* WiFi connection

## Wiring

* **SCL (Clock)**: GPIO 32
* **SDA (Data)**: GPIO 04

## MQTT Topic

* **Publish Topic**: `/Wuerzburg/Kos/A105/data`

## JSON Payload Format

```json
{
  "temperature": 23.56,
  "humidity": 48.23,
  "time": "2025-05-19 14:53:00",
  "pressure": 101325.00,
  "location": "Wuerzburg"
}
```

## Functional Flow

1. **Initialize WiFi and SNTP**: Establishes network connectivity and sets the system time.
2. **Initialize BME280 and Display**: Configures the sensor and clears the display.
3. **Sensor Reading**: Reads and calculates temperature, humidity, and pressure values.
4. **Display Output**: Shows data and current time on the OLED display.
5. **Data Publishing**: Converts data into JSON and sends it to an MQTT broker.
6. **Power Saving**: Puts ESP32 into deep sleep mode for 1 minute.

## Dependencies

* `wifi_handler.h`: Manages WiFi connection.
* `mqtt_handler.h`: Handles MQTT communication.
* `bme280_i2c_driver.h`: Interface for BME280 sensor.
* `display_i2c_driver.h`: OLED display I2C driver.
* `system_time.h`: Time synchronization via SNTP.

## Notes

* There are some issuse with the display driver, it is currently not functional.
* The display should show temperature (°C), humidity (%), pressure (Pa), time (UTC), and a fixed location ("Marienham").
* Deep sleep interval is configurable via `enter_deep_sleep_minutes()`.

