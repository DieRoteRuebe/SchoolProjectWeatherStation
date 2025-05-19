#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <ArduinoJson.h>

// --- Konstante Definitionen ---
#define RETRIES 5
#define SDA_PIN 4       // TDI
#define SCL_PIN 32
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1


struct bme280_data_readout {
  float temp;
  float hum;
  float press;
  bool error;
};

// --- Netzwerk & MQTT-Konfiguration ---
const char* ssid = "xxx"; //insert the ssid for the wifi here
const char* password = "xxx"; //insert the passwort for the wifi here
const char* mqtt_server = "xxx"; //insert the mqtt broker adress here
const char* mqtt_user = "";
const char* mqtt_pass = "";
const int mqtt_port = 1883; //insert the port for the mqtt here
const char* mqtt_sub_topic = "xx"; //insert the Topic for subsription here
const char* mqtt_pub_topic = "xx"; //insert the Topic for publishment here
// --- Weitere Metadaten ---
String location = "xx"; //insert the Location in Long format here
const char* locationShort = "xx"; //insert the location in short here
const char* esp_id = "esp2";
const long interval = 300000;
unsigned long lastMsg = 0;

// --- Globale Objekte ---
Adafruit_BME280 bme;
TwoWire wire = TwoWire(0);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &wire, OLED_RESET);
WiFiClient espClient;
PubSubClient client(espClient);


// --- ESP Functionality ---
void goToDeepSleep(int minutes) {
  Serial.printf("Going to deep sleep for %d minutes...\n", minutes);
  esp_sleep_enable_timer_wakeup((uint64_t)minutes * 60ULL * 1000000ULL);
  esp_deep_sleep_start();
}
//################################
// I2C
// --- Initialisierung I2C-Bus ---
bool initCustomI2CBus(TwoWire* wire, int sclPin, int sdaPin) {
  return wire->begin(sdaPin, sclPin);
}

//################################
// BME280
// --- BME280 Initialisierung ---
bool initBME280(Adafruit_BME280* bme, TwoWire* wire, int address = 0x76) {
  if (wire) {
    return bme->begin(address, wire);
  } else {
    return bme->begin(address);
  }
}

// --- BME280 Functions ---
bme280_data_readout read_bme280_data(Adafruit_BME280* bme) {
  bme280_data_readout retvals;
  retvals.temp = bme->readTemperature();
  retvals.press = bme->readPressure() / 100.0F;
  retvals.hum = bme->readHumidity();
  retvals.error = isnan(retvals.temp) || isnan(retvals.press) || isnan(retvals.hum);
  return retvals;
}


//################################
// DISPLAY
// --- Display Initialisierung ---
bool initDisplay(Adafruit_SSD1306* disp) {
  if (!disp->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 initialisierung fehlgeschlagen");
    return false;
  }
  disp->clearDisplay();
  disp->setTextSize(1);
  disp->setTextColor(SSD1306_WHITE);
  disp->setCursor(10, 25);
  disp->println("Connecting...");
  disp->display();
  return true;
}

// --- Display Functions ---
void displaySensorData(bme280_data_readout data, String timeStr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  char tempStr[20], humStr[20], presStr[25];
  snprintf(tempStr, sizeof(tempStr), "Temp: %.2f C", data.temp);
  snprintf(humStr, sizeof(humStr), "Hum:  %.2f %%", data.hum);
  snprintf(presStr, sizeof(presStr), "Pres: %.2f hPa", data.press);

  int16_t x1, y1;
  uint16_t w, hLine;

  // Location
  display.getTextBounds(location.c_str(), 0, 0, &x1, &y1, &w, &hLine);
  display.setCursor((SCREEN_WIDTH - w) / 2, 5);
  display.println(location);

  // Temperature
  display.getTextBounds(tempStr, 0, 0, &x1, &y1, &w, &hLine);
  display.setCursor((SCREEN_WIDTH - w) / 2, 20);
  display.println(tempStr);

  // Humidity
  display.getTextBounds(humStr, 0, 0, &x1, &y1, &w, &hLine);
  display.setCursor((SCREEN_WIDTH - w) / 2, 30);
  display.println(humStr);

  // Pressure
  display.getTextBounds(presStr, 0, 0, &x1, &y1, &w, &hLine);
  display.setCursor((SCREEN_WIDTH - w) / 2, 40);
  display.println(presStr);

  // Time
  display.getTextBounds(timeStr.c_str(), 0, 0, &x1, &y1, &w, &hLine);
  display.setCursor((SCREEN_WIDTH - w) / 2, 50);
  display.println(timeStr);

  display.display();
}


//################################
// WIFI
// --- WiFi Initialisierung ---
void connectWiFi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// --- WiFi Functions ---
bool wifi_is_connected() {
  if(WiFi.status() != WL_CONNECTED) {
    return false;
  }
  return true;
}


//################################
// TIME
// --- Zeit Initialisierung ---
void syncNTPTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Synchronizing time");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nTime synchronized.");
}

// --- Time Functions ---
String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "1970-01-01 00:00:00";
  }
  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

//################################
// MQTT
// --- MQTT Initialisierung ---
void connectMQTT() {
  while (!client.connected()) {
    String clientId = "ESP32Client-";
    clientId += String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Connecting to MQTT... ");

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(mqtt_sub_topic);
    } else {
      Serial.print("failed (");
      Serial.print(client.state());
      Serial.println("), retrying in 2s...");
      delay(2000);
    }
  }
}

// --- MQTT Functions ---
bool mqtt_is_connected() {
  return client.connected();
}

void mqtt_send_data(bme280_data_readout data, String timestring) {
  // JSON payload
  char payload[300];
  snprintf(payload, sizeof(payload),
           "{\"location\": \"%s\", \"temp\": %.2f, \"hum\": %.2f, \"pressure\": %.2f, \"time\": \"%s\", \"esp_id\": \"%s\"}",
           locationShort, data.temp, data.hum, data.press, timestring.c_str(), esp_id);
  client.publish(mqtt_pub_topic, payload);

  Serial.println("MQTT Payload:");
  Serial.println(payload);
}


//################################
// ESP_SETUP
// --- Setup ---
bool init_had_errors = false;
void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C
  if (initCustomI2CBus(&wire, SCL_PIN, SDA_PIN)) {
    Serial.println("I2C bus initialized");
  } else {
    Serial.println("I2C bus init failed");
    init_had_errors = true;
  
  }

  // Display
  if (!initDisplay(&display)) {
    init_had_errors = true;
    Serial.println("Display init failed");
  }

  // WiFi
  connectWiFi(ssid, password);

  // Zeit
  syncNTPTime();

  // Sensor
  if (!initBME280(&bme, &wire)) {
    init_had_errors = true;
    Serial.println("BME280 not found at 0x76");
  }

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  connectMQTT();
}


//################################
// ESP_LOOP
// --- Loop ---
void loop() {
  client.loop();

  int tries = 0;
  if(!init_had_errors) {
    while(tries < RETRIES) {
      Serial.println("Trying to read sensor and publish data");
        if(wifi_is_connected() && mqtt_is_connected()) {
          bme280_data_readout data = read_bme280_data(&bme);
          if(!data.error) {
            Serial.println("Bme280 read has no error");
            String timestring = getFormattedTime();
            displaySensorData(data, timestring);
            mqtt_send_data(data, timestring);
            break;
          }
        } else {
          //try to reconnect
          Serial.println("Errors occured");
          if(!wifi_is_connected()) {
            connectWiFi(ssid, password);
          } 
          if(!mqtt_is_connected()) {
            connectMQTT();
          }
        }
      tries++;
    }
  }
  goToDeepSleep(5);
}
