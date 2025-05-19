#include <stdio.h>
#include "esp_log.h"
#include "bme280_i2c_driver.h"
#include "mqtt_handler.h"
#include "wifi_handler.h"
#include "system_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_sntp.h"
#include <sys/time.h>
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "display_i2c_driver.h"


#define I2C_MASTER_SCL_IO 32 //SCL -> clock pin (Serial Clock line)
#define I2C_MASTER_SDA_IO 04 //SDA -> data line (Serial data line)

static const char* main_tag = "MAIN";
char json_string[256];

void enter_deep_sleep_minutes(uint32_t minutes) {
    const int64_t wakeup_time_us = minutes * 60LL * 1000000LL;
    esp_sleep_enable_timer_wakeup(wakeup_time_us);
    ESP_LOGI("SLEEP", "Going to sleep for %"PRIu32" minutes", minutes);
    esp_deep_sleep_start();
}

void vTaskDelay_min(uint8_t minutes) {
    vTaskDelay(minutes * 60 * 1000 / portTICK_PERIOD_MS);
}

void create_json_string(char *buffer, size_t buffer_size, bme280 *bme, const char* current_time, const char* location) {
    snprintf(buffer, buffer_size, 
             "{\"temperature\": %.2f, \"humidity\": %.2f, \"time\": \"%s\", \"pressure\": %.2f, \"location\": \"%s\"}", 
             bme280_get_temperature(bme), 
             bme280_get_humidity(bme),
             current_time,
             bme280_get_pressure(bme), 
             location);
}




void app_main(void){
    esp_err_t err = wifi_init();
    if(err == ESP_FAIL){
        //return when wifi_init fails
        ESP_LOGE(main_tag, "[APP] Failed to initialize WiFi");
        return;
    }

    ESP_LOGI(main_tag, "[APP] Startup..");
    ESP_LOGI(main_tag, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(main_tag, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("BME280", ESP_LOG_INFO);
    esp_log_level_set("mqtt_handler", ESP_LOG_INFO);
    esp_log_level_set("wifi_handler", ESP_LOG_INFO);

    bme280 bme;
    ic2_master_init(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    bme280_config conf;
    display_init();
    display_clear();
    
    err = init_bme280(&bme);
    if(err == ESP_OK) {
        ESP_LOGI(main_tag, "BME280 initialized successfully");
        print_addresses(&bme);
    } else {
        ESP_LOGE(main_tag, "Failed to initialize BME280");
        return;
    }
    bme280_set_conf_t(&conf, 2, 3, 1, 2, 0);
    if(bme280_set_config(&conf, bme.sl_address) == ESP_OK) {
        ESP_LOGI(main_tag, "BME280 config set successfully");
    } else {
        ESP_LOGE(main_tag, "Failed to set config for BME280");
        return;
    }
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    // wait for time to be set from NTP server
    if (!init_system_time()) {
        ESP_LOGE(main_tag, "Failed to initialize system time");
        return;
    } 
    ESP_LOGI(main_tag, "Wifi up and running, Https_client ready...");
    if(mqtt_app_start() != ESP_OK) {
        ESP_LOGE(main_tag, "Failed to init mqtt_handler");
        return;
    } else {
        if(!wait_for_connection(10)) {
            return;
        }
    }

    while(1) {
        // read data from BME280
        //err = bme280_wake_up(bme.sl_address, &conf, &bme);
        if(err != ESP_OK) {
            return;
        }
        err = bme280_auto_read_calc(&bme, &conf);
        if(err == ESP_OK) {
            bme280_print_data_raw(&bme);
            bme280_print_calculated_data(&bme);
        } else {
            ESP_LOGE(main_tag, "Failed to read data");
            return;
        }
        double temperature = bme280_get_temperature(&bme);
        double humidity = bme280_get_humidity(&bme);
        double pressure = bme280_get_pressure(&bme);
        char buffer[24];

        char* time = get_current_time_string();
        ESP_LOGI(time_tag, "Aktuelle Zeit: %s" PRIu32, time);
        create_json_string(json_string, sizeof(json_string), &bme, time, "Wuerzburg");
        ESP_LOGI(main_tag, "JSON string: %s", json_string);

        // Zeile 1: Temperatur
        snprintf(buffer, sizeof(buffer), "Temp: %.1f%cC", temperature, 127); // 127 = custom °-Symbol
        display_draw_text(0, 0, buffer, 1);

        // Zeile 2: Luftfeuchtigkeit
        snprintf(buffer, sizeof(buffer), "Hum: %.1f%%", humidity);
        display_draw_text(0, 8, buffer, 1);

        // Zeile 3: Druck
        snprintf(buffer, sizeof(buffer), "Pressure: %.0f Pa", pressure);
        display_draw_text(0, 16, buffer, 1);

        // Zeile 4: Zeit (Datum)
        snprintf(buffer, sizeof(buffer), "Time-UTC+0:");
        display_draw_text(0, 24, buffer, 1);

        // Zeile 5: Datum + Uhrzeit
        //snprintf(buffer, sizeof(buffer), "%04d.%02d.%02d %02d:%02d:%02d", year, month, day, hour, min, sec);
        display_draw_text(0, 32, buffer, 1);

        // Zeile 6: Ort
        display_draw_text(0, 40, "Location: Marienham", 1);

        // Buffer übertragen
        display_update(display_buffer);


        /*
        if(bme280_sleep(bme.sl_address, &bme) != ESP_OK) {
            return;
        }
        */ 
        // create json string


        // send data to mqtt broker
        // wait for 1 minute
        
        if (mqtt_is_connected()) {
            mqtt_publish(get_mqtt_client(), "/Wuerzburg/Kos/A105/data", json_string);
            ESP_LOGI(main_tag, "Data sent to MQTT broker successfully");
        } else {
            ESP_LOGW(main_tag, "MQTT client not connected. Skipping publish.");
        }
        enter_deep_sleep_minutes(1);
    }
}
