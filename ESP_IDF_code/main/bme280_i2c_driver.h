#ifndef BME280DRIVERHPP
#define BME280DRIVERHPP
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdint.h>
#include <string.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
####################################################################
# For Licensing information, please refer to the end of this file. #
####################################################################
For the BME280, we need to include the following libraries:
esp_log.h -> for logging
driver/i2c.h -> for i2c communication
stdint.h -> for uint8_t, uint16_t, uint32_t
string.h -> for memset

How to use this driver:
use the espressif command to create a new project:
idf.py create-project <project_name>
then copy this file into the main folder of the project
then include this file in your main.c file:
-> #include "bme280_i2c_driver.h"
make sure to include the i2c.h via changing the CMakeLists.txt by adding: REQUIRES driver in idf_component_register()
then use the functions in this file to communicate with the BME280 sensor

this driver is mainly for the BME280 sensor with pressure, temperature and humidity
this driver is based on the BME280 datasheet and the BME280 application note
this driver might also work for other sensors with the same BME280 chip

BME280 Manual:
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf

BME280 Datasheet:
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
BME280 Application Note:
https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes/bst-bmp3xx-applicationnote.pdf



BME280 Driver:
ok let me get this straight:
we have two possible Adresses for the Sensor itself:
Sensor id = 0x76 -> sensor is connected to GND
Sensor id = 0x77 -> sensor is connected to VDDIO
-> this means we could potentially connect two sensors on one i2c bus
Sensor Pins:

Vin = 3,3 volts
SDO -> GND
SCL -> SCL
SDA -> SDA

NOTICE:
This driver uses an older version of the i2c protocol.
*/
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000 //this is our BAUD rate -> 100khz -> speed of Communication -> 400khz would be highspeed
#define I2C_MASTER_TIMEOUT_MS 1000 


typedef struct {
    // Temperatur
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    // Druck
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    // Feuchtigkeit
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data;


typedef struct {
    uint8_t osrs_t;     // Temperatur-Oversampling
    uint8_t osrs_p;     // Druck-Oversampling
    uint8_t osrs_h;     // Luftfeuchte-Oversampling
    uint8_t filter;     // IIR-Filter
    uint8_t standby;     // Standby-Zeit
}bme280_config;

typedef struct {
    uint8_t hum_lsb;
    uint8_t hum_msb;
    uint8_t temp_xlsb;
    uint8_t temp_lsb;
    uint8_t temp_msb;
    uint8_t press_xlsb;
    uint8_t press_lsb;
    uint8_t press_msb;
    bool extracted;

}bme280_data_full_raw;

typedef struct {
    int32_t temp;
    uint32_t press;  
    uint32_t hum;
}temp_data;

typedef struct {
    uint8_t chip_id; 
    uint8_t sl_address;

    bme280_calib_data calib_data;       // Kalibrierungsdaten
    bme280_data_full_raw raw_data;      // Rohdaten
    int32_t t_fine;                     // Temperatur-Feinkorrekturwert
    temp_data temp_data;
    bool is_awake;
} bme280;

//Master inits the ESP as a the I2C Master with its defined pins as i2c lanes;
void ic2_master_init(uint8_t sda_io_pin, uint8_t scl_io_pin) {
    //enable i2c protokoll:
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io_pin,
        .scl_io_num = scl_io_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, //SDA needs to be connected with a pullup resistor
        .scl_pullup_en = GPIO_PULLUP_ENABLE, //scl needs to be connected with a pullup resistor
        .master.clk_speed = I2C_MASTER_FREQ_HZ, 
    };

    //Check for errors with i2c_config_t
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}



//if we want to read the entire datas at once -> burst read then size_t should be 8; we will get data as ptr back;
esp_err_t read_bytes(uint8_t reg_addr, uint8_t sl_address, uint8_t* data, size_t len) {
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        sl_address, // depends if the SDO interface is connected to GND || VDDIO => SDO -> GND = 0x76 | SDO -> VDDIO = 0x77 
        &reg_addr,
        1,
        data, //our output data as data* < size_t len
        len,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
}

//just a func to write on the chip iself
esp_err_t write_byte(uint8_t reg_addr, uint8_t sl_address, uint8_t value) {
    //write the value back in buf value
    uint8_t buf[2] = { reg_addr, value };
    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        sl_address,
        buf,
        sizeof(buf),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
}


uint8_t read_chipid(uint8_t chip_id_address, uint8_t sl_address) {
    uint8_t chip_id = 0;
    //0xD0 should be the chip_id adress if not we get -1 anyways or 0xFF or 0x00
    esp_err_t err = read_bytes(chip_id_address, sl_address, &chip_id, 1);

    if(err == ESP_OK && chip_id != 0xFF && chip_id != 0x00) {
        ESP_LOGI("Driver", "chip_id extraced");
        return chip_id;
    }
    ESP_LOGE("DRIVER", "read_chipid: error");
    return 0xFF; //Error happend! -> should not be possible since the most hardwareadresses have only 7 bits
};

bool bme280_find_sl_address(uint8_t register_address_chip_id, uint8_t* sl_address) {
    esp_err_t err;
    uint8_t possible_addresses[2] = { 0x76, 0x77 };
    *sl_address = 0xFF;
    size_t len = 1;
    for(size_t i = 0; i < 2; i++) {
        err = read_bytes(register_address_chip_id, possible_addresses[i], sl_address, len);
        if(err == ESP_OK && *sl_address != 0xFF) {
            ESP_LOGI("DRIVER", "Sl_address found");
            *sl_address = possible_addresses[i];
            return true;
        }
    }
    return false;
};


esp_err_t read_burst_bme280_full(bme280* bme) {
    uint8_t buffer[8];
    esp_err_t err = read_bytes(0xF7, bme->sl_address, buffer, 8);
    if (err == ESP_OK) {
        bme->raw_data.hum_lsb = buffer[7];
        bme->raw_data.hum_msb = buffer[6];
        bme->raw_data.temp_xlsb = buffer[5];
        bme->raw_data.temp_lsb = buffer[4];
        bme->raw_data.temp_msb = buffer[3];
        bme->raw_data.press_xlsb = buffer[2];
        bme->raw_data.press_lsb = buffer[1];
        bme->raw_data.press_msb = buffer[0];
        bme->raw_data.extracted = true;
    } else {
        bme->raw_data.extracted = false;
    }
    return err;
}

esp_err_t bme280_read_calibration_data(uint8_t sl_address, bme280_calib_data* calib) {
    uint8_t calib1[26]; // 0x88 - 0x9F
    uint8_t calib2[7];  // 0xE1 - 0xE7
    uint8_t h1;         // 0xA1

    // 1. Temperature and pressure (0x88 - 0x9F)
    esp_err_t err = read_bytes(0x88, sl_address, calib1, 26);
    if (err != ESP_OK) return err;

    // 2. dig_H1 (0xA1)
    err = read_bytes(0xA1, sl_address, &h1, 1);
    if (err != ESP_OK) return err;

    // 3. Restliche Feuchte (0xE1 - 0xE7)
    err = read_bytes(0xE1, sl_address, calib2, 7);
    if (err != ESP_OK) return err;

    // build the calibration data with the data we got from the sensor
    // Temperature
    calib->dig_T1 = (uint16_t)(calib1[1] << 8 | calib1[0]);
    calib->dig_T2 = (int16_t)(calib1[3] << 8 | calib1[2]);
    calib->dig_T3 = (int16_t)(calib1[5] << 8 | calib1[4]);

    // Pressure
    calib->dig_P1 = (uint16_t)(calib1[7] << 8 | calib1[6]);
    calib->dig_P2 = (int16_t)(calib1[9] << 8 | calib1[8]);
    calib->dig_P3 = (int16_t)(calib1[11] << 8 | calib1[10]);
    calib->dig_P4 = (int16_t)(calib1[13] << 8 | calib1[12]);
    calib->dig_P5 = (int16_t)(calib1[15] << 8 | calib1[14]);
    calib->dig_P6 = (int16_t)(calib1[17] << 8 | calib1[16]);
    calib->dig_P7 = (int16_t)(calib1[19] << 8 | calib1[18]);
    calib->dig_P8 = (int16_t)(calib1[21] << 8 | calib1[20]);
    calib->dig_P9 = (int16_t)(calib1[23] << 8 | calib1[22]);

    // Humidity
    calib->dig_H1 = h1;
    calib->dig_H2 = (int16_t)(calib2[1] << 8 | calib2[0]);
    calib->dig_H3 = calib2[2];

    calib->dig_H4 = (int16_t)((calib2[3] << 4) | (calib2[4] & 0x0F));
    calib->dig_H5 = (int16_t)((calib2[5] << 4) | (calib2[4] >> 4));
    calib->dig_H6 = (int8_t)calib2[6];

    return ESP_OK;
}


//For formulas for compensation and calculation please refer to the manual
int32_t compensate_temperature(int32_t adc_T, bme280_calib_data *calib, int32_t* t_fine) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_T1)) * ((adc_T >> 4) - ((int32_t)calib->dig_T1))) >> 12) * ((int32_t)calib->dig_T3)) >> 14;
    *t_fine = var1 + var2;
    
    T = (*t_fine * 5 + 128) >> 8; // in 0.01°C
    return T;
}

uint32_t compensate_pressure(int32_t adc_P, bme280_calib_data *calib, int32_t t_fine) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3) >> 8) + ((var1 * (int64_t)calib->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib->dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // Avoid division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7) << 4);
    return (uint32_t)(p >> 8); // in Pa
}


uint32_t compensate_humidity(int32_t adc_H, bme280_calib_data *calib, int32_t t_fine) {
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib->dig_H4) << 20) -
                    (((int32_t)calib->dig_H5) * v_x1_u32r)) + 
                   ((int32_t)16384)) >> 15) * 
                (((((((v_x1_u32r * ((int32_t)calib->dig_H6)) >> 10) * 
                    (((v_x1_u32r * ((int32_t)calib->dig_H3)) >> 11) + 
                    ((int32_t)32768))) >> 10) + 
                  ((int32_t)2097152)) * ((int32_t)calib->dig_H2) + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * 
                                 (v_x1_u32r >> 15)) >> 7) * 
                               ((int32_t)calib->dig_H1)) >> 4);

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12); // in 0.001 %RH
}


void calculate_data_temp_press_hum(bme280* bme) {
    int32_t adc_T = (bme->raw_data.temp_msb << 12) | (bme->raw_data.temp_lsb << 4) | (bme->raw_data.temp_xlsb >> 4);
    int32_t adc_P = (bme->raw_data.press_msb << 12) | (bme->raw_data.press_lsb << 4) | (bme->raw_data.press_xlsb >> 4);
    int32_t adc_H = (bme->raw_data.hum_msb << 8) | bme->raw_data.hum_lsb;

    bme->temp_data.temp = compensate_temperature(adc_T, &bme->calib_data, &bme->t_fine); // in 0.01°C
    bme->temp_data.press = compensate_pressure(adc_P, &bme->calib_data, bme->t_fine);  // in Pa
    bme->temp_data.hum = compensate_humidity(adc_H, &bme->calib_data, bme->t_fine);    // in 0.001 %rH
}

//clear the bme280 struct
void bme280_clear(bme280* bme) {
    memset(bme, 0, sizeof(bme280)); // set all values to 0
    bme->sl_address = 0xFF;         // Standard adress should not be possible see the Manual
    bme->chip_id = 0xFF;            // Standard chip_id should not be possible see the Manual 
    bme->t_fine = 0;                // Set back the fine correction value
    // Set the raw data to 0
    bme->raw_data.hum_lsb = 0; 
    bme->raw_data.hum_msb = 0;
    bme->raw_data.temp_xlsb = 0;
    bme->raw_data.temp_lsb = 0;
    bme->raw_data.temp_msb = 0;
    bme->raw_data.press_xlsb = 0;
    bme->raw_data.press_lsb = 0;
    bme->raw_data.press_msb = 0;
    // Set the temp_data to 0
    bme->temp_data.temp = 0;
    bme->temp_data.press = 0;
    bme->temp_data.hum = 0;
    // Set the calibration data to 0
    bme->calib_data.dig_T1 = 0;
    bme->calib_data.dig_T2 = 0;
    bme->calib_data.dig_T3 = 0;
    bme->calib_data.dig_P1 = 0;
    bme->calib_data.dig_P2 = 0;
    bme->calib_data.dig_P3 = 0;
    bme->calib_data.dig_P4 = 0;
    bme->calib_data.dig_P5 = 0;
    bme->calib_data.dig_P6 = 0;
    bme->calib_data.dig_P7 = 0;
    bme->calib_data.dig_P8 = 0;
    bme->calib_data.dig_P9 = 0;
    bme->calib_data.dig_H1 = 0;
    bme->calib_data.dig_H2 = 0;
    bme->calib_data.dig_H3 = 0;
    bme->calib_data.dig_H4 = 0;
    bme->calib_data.dig_H5 = 0;
    bme->calib_data.dig_H6 = 0;
    bme->is_awake = false;
}



/*
currently there are some issues with the sleep function:
its technicly working but the sensor is not going to wake up in time again
esp_err_t bme280_sleep(uint8_t sl_address, bme280* bme) {
    esp_err_t err = ESP_OK;
    if(bme->is_awake) {
        bme->is_awake = false;
        uint8_t config = 0x00; // Sleep mode
        err = write_byte(0xF4, sl_address, config);
        if (err == ESP_OK) {
           ESP_LOGI("DRIVER", "Sensor put to sleep");
        } else {
           ESP_LOGE("DRIVER", "Failed to put sensor to sleep");
        }
    }

    return err;
}
*/

esp_err_t bme280_read_status(bme280* bme) {
    uint8_t status_reg = 0;
    const uint8_t STATUS_REG_ADDR = 0xF3;
    const uint32_t timeout_ms = 1000; // Max 1 Sekunde warten
    uint32_t start_tick = xTaskGetTickCount();
    
    esp_err_t err;

    do {
        err = read_bytes(STATUS_REG_ADDR, bme->sl_address, &status_reg, 1);
        if (err != ESP_OK) {
            ESP_LOGE("BME280", "Failed to read status register");
            return err;
        }

        bool measuring = (status_reg >> 3) & 0x01;

        if (!measuring) {
            ESP_LOGI("DRIVER", "Status read successfull, not messuring");
            return ESP_OK; // Messung abgeschlossen
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms warten, bevor erneut gelesen wird
    } while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms));

    ESP_LOGW("BME280", "Timeout while waiting for measurement to complete");
    return ESP_ERR_TIMEOUT;
}

esp_err_t bme280_wait_for_measurement(uint8_t sl_address, TickType_t timeout_ticks) {
    const TickType_t delay = pdMS_TO_TICKS(10); // 10 ms Delay pro Loop
    TickType_t waited = 0;
    uint8_t status;

    while (waited < timeout_ticks) {
        esp_err_t err = read_bytes(0xF3, sl_address, &status, 1);
        if (err != ESP_OK) return err;

        if ((status & 0x08) == 0) { // Bit 3 = measuring == 0 → fertig
            ESP_LOGI("BME280", "Measurement completed.");
            return ESP_OK;
        }

        vTaskDelay(delay);
        waited += delay;
    }

    ESP_LOGW("BME280", "Timeout waiting for measurement.");
    return ESP_ERR_TIMEOUT;
}



void bme280_print_calculated_data(bme280* bme) {
    printf("Temperature: %.2f °C\n", bme->temp_data.temp / 100.0);
    printf("Pressure: %.2f hPa\n", bme->temp_data.press / 100.0);
    printf("Humidity: %.2f %%\n", bme->temp_data.hum / 1000.0);
}

void bme280_print_data_raw(bme280* bme) {
    printf("Data: hum_lsb %d\nhum_msb %d\ntemp_xlsb %d\ntemp_lsb %d\ntemp_msb %d\n press_xlsb %d\npress_lsb %d\npress_msb %d\n", 
    bme->raw_data.hum_lsb, bme->raw_data.hum_msb, bme->raw_data.temp_xlsb, bme->raw_data.temp_lsb, bme->raw_data.temp_msb, bme->raw_data.press_xlsb, bme->raw_data.press_lsb, bme->raw_data.press_msb);
}


void print_addresses(bme280* bme) {
    printf("Chip-id: %d", bme->chip_id);
    printf("Sl-address: %d", bme->sl_address);
}

double bme280_get_temperature(bme280* bme) {
    return (double)bme->temp_data.temp / 100.0;
}
double bme280_get_pressure(bme280* bme) {
    return (double)bme->temp_data.press / 100.0;
}
double bme280_get_humidity(bme280* bme) {
    return (double)bme->temp_data.hum / 1000.0;
}

esp_err_t bme280_read_data(bme280* bme) {
    esp_err_t err = read_burst_bme280_full(bme);
    if (err != ESP_OK) {
        ESP_LOGE("DRIVER", "Failed to read burst data");
        return err;
    }
    return ESP_OK;
}

esp_err_t bme280_read_data_and_calculate(bme280* bme) {
    esp_err_t err = read_burst_bme280_full(bme);
    if (err != ESP_OK) {
        ESP_LOGE("DRIVER", "Failed to read burst data");
        return err;
    }

    esp_err_t err2 = bme280_read_calibration_data(bme->sl_address, &bme->calib_data);
    if (err2 != ESP_OK) {
        ESP_LOGE("DRIVER", "Failed to read calibration data");
        return err2;
    }

    calculate_data_temp_press_hum(bme);
    return ESP_OK;
}


/*
Mögliche Werte
1. Oversampling-Parameter (osrs_t, osrs_p, osrs_h):
Diese geben an, wie oft ein Wert gemessen wird, um das Ergebnis zu verbessern (mehr Oversampling = bessere Genauigkeit, aber mehr Stromverbrauch und längere Messzeit):

Wert	Bedeutung
0	Oversampling x0 (off)
1	Oversampling x1
2	Oversampling x2
3	Oversampling x4
4	Oversampling x8
5	Oversampling x16

2. Filter (filter):
Aktiviert den internen IIR-Filter (glättet die Daten, nützlich bei schnellen Änderungen):

Wert	Filtergröße
0	Aus
1	2
2	4
3	8
4	16

3. Standby-Zeit (standby):
Gibt an, wie lange der Sensor zwischen zwei Messungen wartet (nur im Normal-Modus relevant):

Wert	Zeit
0	0,5 ms
1	62,5 ms
2	125 ms
3	250 ms
4	500 ms
5	1000 ms
6	10 ms
7	20 ms
*/


// Temperatur x2, Druck x4, Feuchte x1, Filter x4, Standby 250ms
//bme280_set_conf(&conf, 2, 3, 1, 2, 3);
void bme280_set_conf_t(
    bme280_config* conf,
    uint8_t osrs_t,     // Temperatur-Oversampling
    uint8_t osrs_p,     // Druck-Oversampling
    uint8_t osrs_h,     // Luftfeuchte-Oversampling
    uint8_t filter,     // IIR-Filter
    uint8_t standby
) {
    conf->osrs_t = osrs_t;     // Temperatur-Oversampling
    conf->osrs_p = osrs_p;     // Druck-Oversampling
    conf->osrs_h = osrs_h;     // Luftfeuchte-Oversampling
    conf->filter = filter;     // IIR-Filter
    conf->standby = standby;     // Standby-Zeit
}

esp_err_t bme280_set_config(bme280_config* bme280_conf, uint8_t sl_address) {
    uint8_t config = (bme280_conf->standby << 5) | (bme280_conf->filter << 2);
    uint8_t ctrl_meas = (bme280_conf->osrs_t << 5) | (bme280_conf->osrs_p << 2) | 0x03; // Normal mode
    uint8_t ctrl_hum = bme280_conf->osrs_h;

    esp_err_t err = write_byte(0xF2, sl_address, ctrl_hum); // Set humidity oversampling
    if (err != ESP_OK) return err;

    err = write_byte(0xF4, sl_address, ctrl_meas); // Set temperature and pressure oversampling
    if (err != ESP_OK) return err;

    return write_byte(0xF5, sl_address, config); // Set filter and standby time
}

 

esp_err_t bme280_trigger_forced_measurement(uint8_t sl_address) {
    uint8_t ctrl_meas;
    esp_err_t err = read_bytes(0xF4, sl_address, &ctrl_meas, 1);
    if (err != ESP_OK) {
        ESP_LOGE("BME280", "Failed to read ctrl_meas register");
        return err;
    }

    ctrl_meas = (ctrl_meas & 0xFC) | 0x01; // Set mode bits to '01' (forced mode)

    err = write_byte(0xF4, sl_address, ctrl_meas);
    if (err != ESP_OK) {
        ESP_LOGE("BME280", "Failed to write forced mode to ctrl_meas");
        return err;
    }

    ESP_LOGI("BME280", "Forced measurement triggered");
    vTaskDelay(pdMS_TO_TICKS(100)); //hier should be something done with the config vor the oversampling duration
    return ESP_OK;
}

/*
Currently there are some issues with the wake up function:
it is not waking up the sensor in time
there might be also some timing issues with the i2c communication
//wake the sensor up
esp_err_t bme280_wake_up(uint8_t sl_address, bme280_config* conf, bme280* bme) {
    esp_err_t err = ESP_OK;

    if (!bme->is_awake) {
        bme->is_awake = true;

        // Schritt 1: CTRL_HUM setzen (Oversampling Humidity)
        uint8_t ctrl_hum = (conf->osrs_h & 0x07);
        err = write_byte(0xF2, sl_address, ctrl_hum);
        if (err != ESP_OK) {
            ESP_LOGE("DRIVER", "Failed to write CTRL_HUM");
            return err;
        }

        // Schritt 2: CTRL_MEAS setzen (Oversampling Temp & Pressure + Mode)
        uint8_t ctrl_meas = ((conf->osrs_t & 0x07) << 5) |
                            ((conf->osrs_p & 0x07) << 2) |
                            0x01; // Forced Mode (0x03 für Normal, 0x00 für Sleep)
        err = write_byte(0xF4, sl_address, ctrl_meas);
        if (err != ESP_OK) {
            ESP_LOGE("DRIVER", "Failed to write CTRL_MEAS");
            return err;
        }

        // Schritt 3: CONFIG-Register setzen (Filter & Standby Time)
        err = bme280_set_config(conf, sl_address);
        if (err != ESP_OK) {
            ESP_LOGE("DRIVER", "Failed to write CONFIG register");
            return err;
        }

        ESP_LOGI("DRIVER", "Sensor woken up with full config");
    }

    return err;
}
*/

esp_err_t bme280_auto_read_calc(bme280* bme, bme280_config* confg) {
    
    esp_err_t err = bme280_trigger_forced_measurement(bme->sl_address);
    if(err != ESP_OK) {
        ESP_LOGE("DRIVER", "Failed to trigger enforced measurement");
        return err;
    }

    if (bme280_wait_for_measurement(bme->sl_address, pdMS_TO_TICKS(100)) != ESP_OK) {
        ESP_LOGE("DRIVER", "Messung nicht abgeschlossen");
        return ESP_FAIL;
    }
    err = read_burst_bme280_full(bme);
    if (err == ESP_OK) {
        err = bme280_read_calibration_data(bme->sl_address, &bme->calib_data);
        if (err == ESP_OK) {
            calculate_data_temp_press_hum(bme);
            return ESP_OK;
        } else {
            ESP_LOGE("DRIVER", "Failed to read calibration data");
            return err;
        }
    } else {
        ESP_LOGE("DRIVER", "Failed to read burst data");
        return err;
    }
    return ESP_FAIL;
}



esp_err_t init_bme280(bme280* bme) {
    bme280_clear(bme);
    bme->is_awake = true;
    if (bme280_find_sl_address(0xD0, &bme->sl_address)) {
        bme->chip_id = read_chipid(0xD0, bme->sl_address);
        if (bme->chip_id == 0x60) {
            esp_err_t err = bme280_read_calibration_data(bme->sl_address, &bme->calib_data);
            if (err == ESP_OK) {
                bme->is_awake = true;
                ESP_LOGI("BME-INIT", "Initialization successful");
                return ESP_OK;
            } else {
                ESP_LOGE("BME-INIT", "Failed to read calibration data");
                return err;
            }
        } else {
            ESP_LOGE("BME-INIT", "Invalid chip ID");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE("BME-INIT", "BME280 address not found");
        return ESP_FAIL;
    }
}


/*
MIT License

Copyright (c) 2025 DieRoteRuebe

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
Third-Party Licenses

This project makes use of the following third-party libraries:

1. ESP-IDF Framework
- Description: ESP-IDF is the official development framework for the ESP32 chip by Espressif Systems.
- License: Apache License 2.0
- Source: https://github.com/espressif/esp-idf

Apache License 2.0
https://www.apache.org/licenses/LICENSE-2.0

*/

#endif
