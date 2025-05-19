#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "ascii_font.h"  // Enthält das font5x7-Array

#define DISPLAY_ADDRESS 0x3C
#define I2C_MASTER_NUM I2C_NUM_0
#define CMD_MODE  0x00
#define DATA_MODE 0x40

uint8_t display_buffer[128 * 8];

extern const uint8_t font5x7[][5]; // [95][5]

esp_err_t write_byte_disp(uint8_t control_byte, uint8_t data_byte) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control_byte, true);
    i2c_master_write_byte(cmd, data_byte, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE("OLED", "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

void display_update(uint8_t *buffer) {
    for (uint8_t page = 0; page < 8; page++) {
        write_byte_disp(CMD_MODE, 0xB0 + page); // Page address
        write_byte_disp(CMD_MODE, 0x00);        // Column low nibble
        write_byte_disp(CMD_MODE, 0x10);        // Column high nibble

        for (uint8_t col = 0; col < 128; col++) {
            write_byte_disp(DATA_MODE, buffer[page * 128 + col]);
        }
    }
}

void display_init() {
    uint8_t cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F,
        0xD3, 0x00, 0x40, 0xAD, 0x8B,
        0xA1, 0xC8, 0xDA, 0x12, 0x81,
        0x7F, 0xA4, 0xA6, 0x20, 0x00,
        0xAF
    };

    for (int i = 0; i < sizeof(cmds); i++) {
        write_byte_disp(CMD_MODE, cmds[i]);
    }

    memset(display_buffer, 0, sizeof(display_buffer));
    display_update(display_buffer);
}

void display_clear() {
    memset(display_buffer, 0, sizeof(display_buffer));
    display_update(display_buffer);
}



void display_draw_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= 128 || y >= 64) return;

    uint16_t index = (y / 8) * 128 + x;
    if (color) {
        display_buffer[index] |= (1 << (y % 8));
    } else {
        display_buffer[index] &= ~(1 << (y % 8));
    }
}

void display_draw_char(uint8_t x, uint8_t y, char c, uint8_t color) {
    if (c < 32 || c > 126) return;
    const uint8_t *bitmap = font5x7[c - 32];

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = bitmap[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t pixel_on = line & (1 << j);
            display_draw_pixel(x + i, y + j, pixel_on ? color : !color);
        }
    }
}

void display_draw_text(uint8_t x, uint8_t y, const char *text, uint8_t color) {
    while (*text && x < 128 - 6) {
        display_draw_char(x, y, *text++, color);
        x += 6;
    }
}
