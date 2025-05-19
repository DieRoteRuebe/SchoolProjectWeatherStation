#include "esp_sntp.h"
#include "esp_log.h"
#include <time.h>
#include <stdio.h>

static const char *time_tag = "SYSTEM_TIME";



char* get_current_time_string() {
    static char buffer[64]; // statisch, damit sie nach Funktionsende noch gültig ist
    time_t now;
    struct tm timeinfo;

    time(&now); // aktuelle Zeit holen
    localtime_r(&now, &timeinfo); // in lokale Zeitstruktur umwandeln

    // Formatieren: z.B. "2025-05-08 14:23:45"
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);

    return buffer;
}


bool init_system_time() {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    // Warte auf die Synchronisierung der Zeit
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;

    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(time_tag, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGE(time_tag, "Failed to set system time");
        return false;
    }

    ESP_LOGI(time_tag, "System time is set");
    return true;
}
