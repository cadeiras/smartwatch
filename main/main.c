#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"

#include "lv_demos.h"
//#include "ebike/lv_demo_ebike.h"
//#include "smartwatch/lv_demo_smartwatch.h"
#include "smartwatch_libs/smartwatch_libs.h"

// LCD controller drivers
#include "esp_lcd_sh8601.h"

#include "lvgl.h"

// LCD Touch 
#include "esp_lcd_touch_ft5x06.h"
// Altere para o endereço do FT3168 (0x48)

#include <stdio.h>


static const char *TAG = "smartwatch";

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t []){0x00}, 0, 80},   
    {0xC4, (uint8_t []){0x80}, 1, 0},
   
    {0x35, (uint8_t []){0x00}, 1, 0},

    {0x53, (uint8_t []){0x20}, 1, 1},
    {0x63, (uint8_t []){0xFF}, 1, 1},
    {0x51, (uint8_t []){0x00}, 1, 1},

    {0x29, (uint8_t []){0x00}, 0, 10},

    {0x51, (uint8_t []){0xFF}, 1, 0},    //亮度
};

static void smartwatch_touch_read(lv_indev_t * indev, lv_indev_data_t * data) {
    esp_lcd_touch_handle_t handle = (esp_lcd_touch_handle_t)lv_indev_get_user_data(indev);
    
    // VERIFICAÇÃO CRUCIAL: Se o handle for NULL, sai imediatamente
    if (handle == NULL) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    uint16_t x, y;
    uint8_t points;
    if (esp_lcd_touch_read_data(handle) == ESP_OK) {
        if (esp_lcd_touch_get_coordinates(handle, &x, &y, NULL, &points, 1)) {
            data->point.x = x;
            data->point.y = y;
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void app_main(void) {

    static esp_lcd_panel_io_handle_t io_handle = NULL;
    static lv_disp_t * disp_driver = NULL;

    // 1. Configurar o Barramento SPI para modo Quad
    ESP_LOGI(TAG, "Configure SPI bus for QSPI interface");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(   EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                    EXAMPLE_PIN_NUM_LCD_DATA1, EXAMPLE_PIN_NUM_LCD_DATA2,
                                                                    EXAMPLE_PIN_NUM_LCD_DATA3,
                                                                    EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 2 );

    // No S3, o SPI2 (FSPI) ou SPI3 podem ser usados
    ESP_LOGI(TAG, "Initialize SPI bus");
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, NULL, &disp_driver);
    //const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, example_notify_lvgl_flush_ready, &disp_driver);
    ESP_LOGI(TAG, "Create new panel IO");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) EXAMPLE_LCD_HOST, &io_config, &io_handle));

    // 2. CONFIGURAÇÃO DO VENDOR (SH8601)
    ESP_LOGI(TAG, "Configure SH8601 vendor specific configuration");
    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    ESP_LOGI(TAG, "Configure SH8601 panel configuration");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_LOGI(TAG, "Create new SH8601 panel");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));

    ESP_LOGI(TAG, "Initialize panel hardware");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 20, 0));

    ESP_LOGI(TAG, "Initialize LVGL port");
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,
        .task_stack = 24576,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 20,
    };

    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    ESP_LOGI(TAG, "Add display to LVGL port");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 4,
        .double_buffer = 1,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = 0,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .flags = {
            .buff_spiram = 1, // MUITO IMPORTANTE: Tens 8MB de PSRAM, usa-a!
            .buff_dma = 1,
            .swap_bytes = 1,
            .sw_rotate = 0,
        }
    };
    disp_driver = lvgl_port_add_disp(&disp_cfg);

    ESP_LOGI(TAG, "Display added to LVGL port");
    i2c_master_bus_handle_t i2c_bus_handle;
    const i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = EXAMPLE_PIN_NUM_TOUCH_SCL,
        .sda_io_num = EXAMPLE_PIN_NUM_TOUCH_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x58,
        .scl_speed_hz = 300 * 1000,
        .scl_wait_us = 0, // <--- ADICIONA ISTO (Aumenta espera para 20ms)
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle));

    // No bloco de inicialização do Touch
    ESP_LOGI(TAG, "Configure FT5x06 touch controller");
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    const esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = 0x38,
        .scl_speed_hz = 300 * 1000,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 0,
        .flags = {
            .disable_control_phase = 1, // Tente ativar isto se o NACK continuar
        }
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(i2c_bus_handle, &tp_io_config, &tp_io_handle));

    ESP_LOGI(TAG, "Create FT5x06 touch handle");
    esp_lcd_touch_handle_t tp_handle = NULL;
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = 6,
        .int_gpio_num = -1,
        .levels = { .reset = 0, .interrupt = 0 }, // Tente 0 primeiro
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0, // Tente 1 para alinhar com o AMOLED
            .mirror_y = 0, // Tente 1 para alinhar com o AMOLED
        },
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp_handle));

    /*
    ESP_LOGI(TAG, "Add touch to LVGL port");
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp_driver,
        .handle = tp_handle,
    };
    lvgl_port_add_touch(&touch_cfg);
    */
    
    // No seu app_main, APÓS criar o tp_handle:
    ESP_LOGI(TAG, "Registando touch manualmente no LVGL 9...");
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        lv_indev_t * indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, smartwatch_touch_read);
        lv_indev_set_user_data(indev, tp_handle);
        lv_indev_set_display(indev, disp_driver);
        lvgl_port_unlock();
    }

    // --- 5. CRIAR CONTEÚDO (Sem loops infinitos aqui) ---
    ESP_LOGI(TAG, "Create LVGL main content");
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        lv_demo_music();
        //lv_demo_widgets();
        //lv_demo_smartwatch();
        //lv_demo_ebike();
        lvgl_port_unlock();
        // Substitua o seu while(1) final por este:
        ESP_LOGI(TAG, "Touch polling loop started");
    }
   // Turn off backlight before LCD initialization
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };

    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
}