#include <stdio.h>
#include "lvgl_setup.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ui_helpers.h"

static const char *TAG = "LVGL_SETUP";
static void lvgl_timer_task(void *arg);

static lv_obj_t *ui_Screen1;
static lv_obj_t *ui_redsquare;

static lv_obj_t *meter;
static lv_obj_t *ue_img_logo;
static lv_obj_t *esp_img_logo;

static lv_obj_t *ui_Dropdown2;

LV_IMG_DECLARE(ue_logo)
LV_IMG_DECLARE(esp_logo)
LV_IMG_DECLARE(red_square)

#define BSP_NULL_CHECK(x, ret) assert(x)

static SemaphoreHandle_t lvgl_mux;  // LVGL mutex
static SemaphoreHandle_t touch_mux; // Touch mutex

static void bsp_touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read data from touch controller into memory */
    if (xSemaphoreTake(touch_mux, 0) == pdTRUE)
    {
        esp_lcd_touch_read_data(drv->user_data);
    }
    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        printf("data->point.x = %u \n", data->point.x);
        printf("data->point.y = %u \n", data->point.y);
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void touch_callback(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void set_value(void *indic, int32_t v)
{
    lv_meter_set_indicator_end_value(meter, indic, v);
}

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void lvgl_setup()
{

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    gpio_config_t pwr_gpio_config =
        {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_POWER};
    ESP_ERROR_CHECK(gpio_config(&pwr_gpio_config));
    gpio_set_level(EXAMPLE_PIN_NUM_POWER, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    gpio_config_t input_conf =
        {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pin_bit_mask = 1ULL << PIN_LCD_RD};
    ESP_ERROR_CHECK(gpio_config(&input_conf));

    gpio_config_t bk_gpio_config =
        {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
        .wr_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
        },
        .bus_width = 8,
        .max_transfer_bytes = LVGL_LCD_BUF_SIZE * sizeof(uint16_t)
        //.psram_trans_align = EXAMPLE_PSRAM_DATA_ALIGNMENT,
        //.sram_trans_align = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 20,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },

        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;

    ESP_LOGI(TAG, "Install LCD driver of st7789");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        .rgb_endian = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);

    esp_lcd_panel_swap_xy(panel_handle, true);

    // WITHOUT 180 DEGREES ROTATION
    // esp_lcd_panel_mirror(panel_handle, true, false); //Y AXIS SHIFTED BUT X AXIS OK
    // esp_lcd_panel_mirror(panel_handle, true, true); //Y AXIS SHIFTED AND X AXIS SHIFTED
    // esp_lcd_panel_mirror(panel_handle, false, false); //Everything is ok but its upside down
    esp_lcd_panel_mirror(panel_handle, false, true); // Not upside down anymore but now the X axis is inverted

    // the gap is LCD panel specific, even panels with the same driver IC, can have different gap value
    esp_lcd_panel_set_gap(panel_handle, 0, 35);

    esp_lcd_panel_io_tx_param(io_handle, 0xF2, (uint8_t[]){0}, 1); // 3Gamma function disable
    esp_lcd_panel_io_tx_param(io_handle, 0x26, (uint8_t[]){1}, 1); // Gamma curve 1 selected
    esp_lcd_panel_io_tx_param(io_handle, 0xE0, (uint8_t[]){        // Set positive gamma
                                                           0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00},
                              15);
    esp_lcd_panel_io_tx_param(io_handle, 0xE1, (uint8_t[]){// Set negative gamma
                                                           0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F},
                              15);

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if USE_TOUCH_DISPLAY
    // CST816 TOUCH TESTING FUNCTION
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_I2C_SDA,
        .scl_io_num = EXAMPLE_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_LOGI(TAG, "Initializing I2C for display touch");
    /* Initialize I2C */
    ESP_ERROR_CHECK(i2c_param_config(EXAMPLE_I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(EXAMPLE_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    i2c_cmd_handle_t cmd;
    for (int i = 0; i < 0x7f; i++)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(EXAMPLE_I2C_NUM, cmd, portMAX_DELAY) == ESP_OK)
        {
            ESP_LOGW("I2C_TEST", "%02X", i);
        }
        i2c_cmd_link_delete(cmd);
    }

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();

    ESP_LOGI(TAG, "esp_lcd_new_panel_io_i2c");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_I2C_NUM, &tp_io_config, &tp_io_handle));
    
        esp_lcd_touch_config_t tp_cfg = {
        .x_max = 170,
        .y_max = 320,
        .rst_gpio_num = 21,
        .int_gpio_num = 16,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 1,
            .mirror_x = 0,
            .mirror_y = 1,
        },
        .interrupt_callback = touch_callback,
    };


    ESP_LOGI(TAG, "esp_lcd_touch_new_i2c_cst816s");
    esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);
    // END OF CST816 TOUCH TESTING FUNCTION
#endif

    lv_init();

    lvgl_mux = xSemaphoreCreateMutex();
    BSP_NULL_CHECK(lvgl_mux, NULL);

    touch_mux = xSemaphoreCreateBinary();
    BSP_NULL_CHECK(touch_mux, NULL);

    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, LVGL_LCD_BUF_SIZE);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.sw_rotate = 1;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if USE_TOUCH_DISPLAY
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_t *indev_touchpad;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = bsp_touchpad_read;
    indev_drv.user_data = tp;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
    BSP_NULL_CHECK(indev_touchpad, ESP_ERR_NO_MEM);
#endif

    xTaskCreatePinnedToCore(lvgl_timer_task, "lvgl Timer", 10000, NULL, 4, NULL, 1);
}

static void lvgl_timer_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    while (1)
    {
        bsp_display_lock(0);
        uint32_t task_delay_ms = lv_timer_handler();
        bsp_display_unlock();
        if (task_delay_ms > 500)
        {
            task_delay_ms = 500;
        }
        else if (task_delay_ms < 5)
        {
            task_delay_ms = 5;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void display_meter()
{
    meter = lv_meter_create(lv_scr_act());
    lv_obj_center(meter);
    lv_obj_set_size(meter, 170, 170);

    /*Remove the circle from the middle*/
    lv_obj_remove_style(meter, NULL, LV_PART_INDICATOR);

    /*Add a scale first*/
    lv_meter_scale_t *scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 11, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 1, 2, 15, lv_color_hex3(0xeee), 10);
    lv_meter_set_scale_range(meter, scale, 0, 100, 270, 90);

    /*Add a three arc indicator*/
    lv_meter_indicator_t *indic1 = lv_meter_add_arc(meter, scale, 10, lv_color_hex3(0x00F), 0);
    // lv_meter_indicator_t *indic2 = lv_meter_add_arc(meter, scale, 10, lv_color_hex3(0x0F0), -10);
    // lv_meter_indicator_t *indic3 = lv_meter_add_arc(meter, scale, 10, lv_color_hex3(0xF00), -20);

    /*Create an animation to set the value*/
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, set_value);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_repeat_delay(&a, 100);
    lv_anim_set_playback_delay(&a, 2000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);

    lv_anim_set_time(&a, 2000);
    lv_anim_set_playback_time(&a, 500);
    lv_anim_set_var(&a, indic1);
    lv_anim_start(&a);

    /*
        lv_anim_set_time(&a, 1000);
        lv_anim_set_playback_time(&a, 1000);
        lv_anim_set_var(&a, indic2);
        lv_anim_start(&a);

        lv_anim_set_time(&a, 1000);
        lv_anim_set_playback_time(&a, 2000);
        lv_anim_set_var(&a, indic3);
        lv_anim_start(&a);
        */
    /* page 3 */
}

static void dropdown_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
        char buf[32];
        lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
        LV_LOG_USER("Option: %s", buf);
    }
}

/**
 * @brief dispaly dropdown is rotated 180 degrees (USB to the right is the orentation)
 *
 */
void display_dropdown()
{
    lv_disp_t *dispp = lv_disp_get_default();
    ui_Dropdown2 = lv_dropdown_create(lv_scr_act());
    lv_dropdown_set_options(ui_Dropdown2, "Apple\n"
                                          "Banana\n"
                                          "Orange\n"
                                          "Melon\n"
                                          "Grape\n"
                                          "Raspberry");
    lv_obj_set_width(ui_Dropdown2, 150);
    lv_obj_set_height(ui_Dropdown2, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_Dropdown2, 0);
    lv_obj_set_y(ui_Dropdown2, -20);
    lv_obj_set_align(ui_Dropdown2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown2, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_add_event_cb(ui_Dropdown2, dropdown_event_handler, LV_EVENT_ALL, NULL);
}

void display_window()
{
    lv_obj_t *win = lv_win_create(lv_scr_act(), 40);
    assert(win);
    lv_win_add_title(win, "test123!");
}

void display_image()
{
    esp_img_logo = lv_img_create(lv_scr_act());
    lv_img_set_src(esp_img_logo, &esp_logo);
    lv_obj_center(esp_img_logo);
}

void display_red_square()
{
    // ui_Screen1 = lv_obj_create(NULL);
    // lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_redsquare = lv_img_create(lv_scr_act());
    lv_img_set_src(ui_redsquare, &red_square);
    lv_obj_set_width(ui_redsquare, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_redsquare, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_redsquare, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_redsquare, LV_OBJ_FLAG_ADV_HITTEST);  /// Flags
    lv_obj_clear_flag(ui_redsquare, LV_OBJ_FLAG_SCROLLABLE); /// Flags
}

static void slider_event_cb(lv_event_t *e);
static lv_obj_t *slider_label;

void display_slider()
{
    lv_disp_t *dispp = lv_disp_get_default();
    /*Create a slider in the center of the display*/
    lv_obj_t *slider = lv_slider_create(lv_scr_act());
    lv_obj_set_x(slider, 0);
    lv_obj_set_y(slider, -40);
    lv_obj_set_align(slider, LV_ALIGN_CENTER);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /*Create a label below the slider*/
    slider_label = lv_label_create(lv_scr_act());
    lv_label_set_text(slider_label, "0%");
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

static void slider_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    lv_label_set_text(slider_label, buf);
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

void get_img_color()
{
    lv_color_t pixel_color = lv_img_buf_get_px_color(&red_square, 50, 50, lv_color_make(0, 0, 0));
    uint32_t c32 = lv_color_to32(pixel_color);
    printf("Pixel color: %lu \n", c32);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    BSP_NULL_CHECK(lvgl_mux, NULL);
    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

void bsp_display_unlock(void)
{
    BSP_NULL_CHECK(lvgl_mux, NULL);
    xSemaphoreGive(lvgl_mux);
}
