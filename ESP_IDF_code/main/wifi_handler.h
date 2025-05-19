#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define WIFI_SSID      "FBIT.IoT.Router7"  //FBIT.IoT.Router1
#define WIFI_PASSWORD  "WueLoveIoT"     //WueLoveIoT

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_GOT_IP_BIT    BIT1 


static EventGroupHandle_t wifi_event_group;
static const char *wifi_tag = "Wifi_handler";


// WiFi Event Handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(wifi_tag, "Verbinde mit WLAN...");
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(wifi_tag, "Mit WLAN verbunden!");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);      //set event bit für connection
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        esp_netif_ip_info_t *ip_info = (esp_netif_ip_info_t *)event_data;
        ESP_LOGI(wifi_tag, "IP-Adresse: " IPSTR, IP2STR(&ip_info->ip));
        xEventGroupSetBits(wifi_event_group, WIFI_GOT_IP_BIT);         //set event bit für ip
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(wifi_tag, "Verbindung verloren! Erneuter Versuch...");
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);    //clear event bits
        esp_wifi_connect();
    }
}

// WiFi Initialisierung
void wifi_init_sta() {
    wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta(); // Erstellt das Standard WiFi-Interface
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(wifi_tag, "Fehler bei der WiFi-Initialisierung: %s", esp_err_to_name(err));
        return;
    }

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);  // Station-Mode aktivieren
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(wifi_tag, "Fehler beim starten des WiFi: %s", esp_err_to_name(err));
    }
}

esp_err_t wifi_init() {
    ESP_LOGI(wifi_tag, "Starte WiFi Client...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_LOGE(wifi_tag, "Fehler bei NVS Flash Initialisierung, flash gelöscht und erneuter versuch: %s", esp_err_to_name(err));
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(wifi_tag, "Fehler bei NVS Flash Initialisierung: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_sta();
    return ESP_OK;
}
