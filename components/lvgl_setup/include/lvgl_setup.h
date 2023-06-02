
#ifndef LVGL_SETUP_H
#define LVGL_SETUP_H

#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (10 * 1000 * 1000)

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_DATA0 39    // 6
#define EXAMPLE_PIN_NUM_DATA1 40    // 7
#define EXAMPLE_PIN_NUM_DATA2 41    // 8
#define EXAMPLE_PIN_NUM_DATA3 42    // 9
#define EXAMPLE_PIN_NUM_DATA4 45    // 10
#define EXAMPLE_PIN_NUM_DATA5 46    // 11
#define EXAMPLE_PIN_NUM_DATA6 47    // 12
#define EXAMPLE_PIN_NUM_DATA7 48    // 13
#define EXAMPLE_PIN_NUM_PCLK 8      // 5
#define EXAMPLE_PIN_NUM_CS 6        // 3
#define EXAMPLE_PIN_NUM_DC 7        // 4
#define EXAMPLE_PIN_NUM_RST 5       // 2
#define EXAMPLE_PIN_NUM_BK_LIGHT 38 // 1
#define EXAMPLE_PIN_NUM_POWER 15
#define PIN_LCD_RD 9

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 320
#define EXAMPLE_LCD_V_RES 170
#define LVGL_LCD_BUF_SIZE            (EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES)
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// Supported alignment: 16, 32, 64. A higher alignment can enables higher burst transfer size, thus a higher i80 bus throughput.
#define EXAMPLE_PSRAM_DATA_ALIGNMENT 32


void lvgl_setup();
void display_meter();
void display_window();
void display_image();





#endif /* LVGL_SETUP_H */