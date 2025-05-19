# Third-Party Libraries Used

This project relies on several open-source libraries to enable sensor communication, display output, networking, and data serialization. Below is a list of all third-party libraries used, including a short description and links to their respective repositories or documentation.

1. WiFi.h

   * Description: Enables Wi-Fi connectivity on the ESP32 microcontroller.
   * Platform: Built-in for ESP32 via the Arduino core
   * Link: [https://github.com/espressif/arduino-esp32](https://github.com/espressif/arduino-esp32)

2. Wire.h

   * Description: Standard Arduino I2C library used for communication with I2C devices.
   * Platform: Arduino Core
   * Link: [https://www.arduino.cc/en/Reference/Wire](https://www.arduino.cc/en/Reference/Wire)

3. Adafruit Sensor Library (Adafruit\_Sensor.h)

   * Description: Common sensor interface library used with Adafruit sensor drivers.
   * Author: Adafruit
   * Link: [https://github.com/adafruit/Adafruit\_Sensor](https://github.com/adafruit/Adafruit_Sensor)

4. Adafruit BME280 Library (Adafruit\_BME280.h)

   * Description: Library for reading temperature, humidity, and pressure data from BME280 sensors.
   * Author: Adafruit
   * Link: [https://github.com/adafruit/Adafruit\_BME280\_Library](https://github.com/adafruit/Adafruit_BME280_Library)

5. Adafruit GFX Library (Adafruit\_GFX.h)

   * Description: Graphics library for drawing shapes and text; used with OLED display libraries.
   * Author: Adafruit
   * Link: [https://github.com/adafruit/Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library)

6. Adafruit SSD1306 Library (Adafruit\_SSD1306.h)

   * Description: OLED driver library for SSD1306-based displays.
   * Author: Adafruit
   * Link: [https://github.com/adafruit/Adafruit\_SSD1306](https://github.com/adafruit/Adafruit_SSD1306)

7. PubSubClient (PubSubClient.h)

   * Description: Lightweight MQTT client library for Arduino. Enables ESP32 to publish and subscribe to topics.
   * Author: Nick O'Leary
   * Link: [https://github.com/knolleary/pubsubclient](https://github.com/knolleary/pubsubclient)

8. ArduinoJson (ArduinoJson.h)

   * Description: Efficient JSON serialization and deserialization for embedded systems.
   * Author: Benoît Blanchon
   * Link: [https://arduinojson.org/](https://arduinojson.org/)

9. time.h

   * Description: Standard C library for time management, used here for syncing with NTP.
   * Platform: Included with ESP32 Arduino Core
   * Link: [https://www.cplusplus.com/reference/ctime/](https://www.cplusplus.com/reference/ctime/)

10. ESP-IDF Framework
   * Description: ESP-IDF is the official development framework for the ESP32 chip by Espressif Systems.
   * License: Apache License 2.0
   * Source: https://github.com/espressif/esp-idf

Apache License 2.0
https://www.apache.org/licenses/LICENSE-2.0

Notes

* All libraries are open source and compatible with the Arduino IDE.
* Some libraries (like Adafruit's) require installing dependencies via the Arduino Library Manager.
* Ensure you have the latest versions installed for best compatibility.

Let me know if you'd like a German version or to include this in a combined document (like a report or full documentation bundle).
