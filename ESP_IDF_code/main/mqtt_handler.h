#include "mqtt_client.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <inttypes.h>  // für PRIi32

static const char *mqtt_handler = "mqtt_handler";
extern char json_string[];

static esp_mqtt_client_handle_t static_client_handle = NULL;
bool is_connected = false;

esp_mqtt_client_handle_t get_mqtt_client() {
    return static_client_handle;
}

bool mqtt_is_connected() {
    return static_client_handle != NULL && is_connected;
}

bool wait_for_connection(uint8_t timeout_seconds) {
    const char* TAG = "MQTT_HANDLER";
    ESP_LOGI(TAG, "Trying to connect...");

    while (!is_connected && timeout_seconds > 0) {
        ESP_LOGI(TAG, "Not connected, trying again... (%d" PRIu32 " seconds left)", timeout_seconds);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Warte 1 Sekunde
        timeout_seconds--;
    }

    if (is_connected) {
        ESP_LOGI(TAG, "Connection successful!");
        return true;
    } else {
        ESP_LOGW(TAG, "Connection timed out.");
        return false;
    }
}

esp_err_t mqtt_publish(esp_mqtt_client_handle_t client, const char *topic, const char *data) {
    int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(mqtt_handler, "Failed to publish message");
        return ESP_FAIL;
    }
    ESP_LOGI(mqtt_handler, "Message published successfully, msg_id=%d" PRId32, msg_id);
    return ESP_OK;
}

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        //ESP_LOGE(mqtt_handler, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    ESP_LOGI(mqtt_handler, "MQTT Ereignis erhalten: %" PRId32, event_id);  // <- HIER

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(mqtt_handler, "MQTT connected");
        is_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(mqtt_handler, "MQTT disconnected");
        is_connected = false;
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(mqtt_handler, "MQTT error");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("socket errno", event->error_handle->esp_transport_sock_errno);
        }
        break;
    default:
        break;
    }
}

esp_err_t mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.108.14:1883",
    };

    static_client_handle = esp_mqtt_client_init(&mqtt_cfg);
    if (!static_client_handle) {
        ESP_LOGE(mqtt_handler, "Failed to init MQTT client");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(static_client_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(static_client_handle);
    if (err != ESP_OK) {
        ESP_LOGE(mqtt_handler, "Failed to start MQTT client: %s" PRId32, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(mqtt_handler, "MQTT client started");
    return ESP_OK;
}
