#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lv_adapter.h"

#include "lv_demos.h"
#include "ebike/lv_demo_ebike.h"
#include "smartwatch/lv_demo_smartwatch.h"
#include "smartwatch_libs/smartwatch_libs.h"

// LCD controller drivers
#include "esp_lcd_sh8601.h"

// LCD Touch 
#include "esp_lcd_touch_ft5x06.h"

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

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    
    if (rotation != LV_DISPLAY_ROTATION_0) {
        // 1. Criar uma área rotacionada baseada na área original
        lv_area_t rotated_area = *area;
        lv_display_rotate_area(disp, &rotated_area);

        // 2. Alocar ou usar um buffer temporário para a rotação
        // O LVGL 9 fornece lv_draw_sw_rotate para facilitar este processo
        lv_color_format_t cf = lv_display_get_color_format(disp);
        uint32_t stride = lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
        
        // Exemplo simplificado: você precisará de um buffer de destino rotacionado
        // Alguns adaptadores da Espressif já gerenciam isso se usar a 'esp_lvgl_port'
    }

    // Enviar para o hardware (ex: esp_lcd_panel_draw_bitmap)
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
    lv_display_flush_ready(disp);
}

bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    ESP_LOGI(TAG, "Getting the disp_driver and executing lv_disp_flush_ready");
    lv_display_t *disp_driver = (lv_display_t *) user_ctx;
    lv_disp_flush_ready(disp_driver);
    ESP_LOGI(TAG, "Got the disp_driver and executing lv_disp_flush_ready");
    return true;
}

// Callback de leitura para v9 e ESP-IDF
void my_touchpad_read(lv_indev_t * indev, lv_indev_data_t * data) {
    if (gpio_get_level(EXAMPLE_PIN_NUM_TOUCH_INT) != 0) { 
         data->state = LV_INDEV_STATE_RELEASED;
         return; 
    }
    // Recupera o handle do esp_lcd_touch
    esp_lcd_touch_handle_t touch_handle = lv_indev_get_user_data(indev);
    if (touch_handle == NULL) {
        ESP_LOGI(TAG, "Touch Handle is null, nothing to do here");
        return;
    } 
    
    uint16_t x[1], y[1];
    uint8_t touch_cnt = 0;

    // Lê os dados do hardware via driver IDF
    esp_lcd_touch_read_data(touch_handle);

    bool pressed = esp_lcd_touch_get_coordinates(touch_handle, x, y, NULL, &touch_cnt, 1);

    if (pressed && touch_cnt > 0) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x[0];
        data->point.y = y[0];
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void app_main(void)
{
    static lv_display_t *disp_driver = NULL;

    // Turn off backlight before LCD initialization
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    // Initialize SPI bus
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(   EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                    EXAMPLE_PIN_NUM_LCD_DATA1, EXAMPLE_PIN_NUM_LCD_DATA2,
                                                                    EXAMPLE_PIN_NUM_LCD_DATA3,
                                                                    EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t) );

    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Install panel IO
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, NULL, &disp_driver);
    //const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, example_notify_lvgl_flush_ready, &disp_driver);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) EXAMPLE_LCD_HOST, &io_config, &io_handle));

    // Configure vendor-specific settings
    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    // Install LCD driver
    ESP_LOGI(TAG, "Install LCD driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Initialize touch controller if enabled
//#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
    ESP_LOGI(TAG, "Initialize touch controller");
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle));

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT3168_CONFIG();
    ESP_LOGI("DEBUG", "Freq: %lu", tp_io_config.scl_speed_hz);

    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT3168_ADDRESS,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 0,
        .scl_speed_hz = 100000,
    }; 
    

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = -1, // Ajuste conforme o seu hardware
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
    };

    esp_lcd_touch_handle_t tp_handle = NULL;
    //ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &tp_io_config, &tp_io_handle));

    ESP_LOGI("DEBUG", "Freq: %lu", tp_io_config.scl_speed_hz);
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(i2c_bus_handle, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp_handle));
    
    // Initialize LVGL adapter
    ESP_LOGI(TAG, "Initialize LVGL adapter");
    const esp_lv_adapter_config_t adapter_config = EBIKE_LV_ADAPTER_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_lv_adapter_init(&adapter_config));

    // Register display (LCD with GRAM uses SPI with PSRAM default config)
    esp_lv_adapter_display_config_t display_config = ESP_LV_ADAPTER_DISPLAY_SPI_WITH_PSRAM_DEFAULT_CONFIG(
                                                         panel_handle,
                                                         io_handle,
                                                         EXAMPLE_LCD_H_RES,
                                                         EXAMPLE_LCD_V_RES,
                                                         ESP_LV_ADAPTER_ROTATE_0  // Rotation not supported for QSPI LCD
                                                     );
    lv_display_t *disp = esp_lv_adapter_register_display(&display_config);
    //lv_display_t * disp = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565); 
    //lv_display_set_user_data(disp, panel_handle); 
    //lv_display_set_flush_cb(disp, example_lvgl_flush_cb);

    // 4. (Opcional) Definir a rotação se for usar a lógica de rotação manual no seu callback
    //lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
    
    //lv_display_set_matrix_rotation(disp, false);
    //lv_display_set_physical_resolution(disp,EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    //lv_display_set_resolution(disp,EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    //lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
    //ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false)); 
    //ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    //lv_display_t *display = lv_display_get_default();

    // Opções: LV_DISPLAY_ROTATION_0, _90, _180, _270
    //lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
    // Tente forçar o modo Landscape no hardware
    
    //lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
    //lv_display_set_color_format(LV_COLOR_FORMAT_RGB565);
    //lv_display_add_event_cb(disp, my_rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    
    // 1. Declare apenas os ponteiros fora ou dentro da função (como preferir)
    static uint16_t *buf1 = NULL;
    static uint16_t *buf2 = NULL; // Recomendado para performance (Double Buffering)

    // 2. Aloque a memória dinamicamente (dentro do setup ou função init)
    size_t buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(uint8_t) * 3;

    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // Verifique se a alocação falhou (se for muito grande para a SRAM interna)
    if (buf1 == NULL) {
        ESP_LOGI(TAG, "Falhou na alocação de memória no buf1");
        ESP_LOGE("LVGL", "Falha ao alocar buffer na PSRAM!");
        // Se falhar, tenta alocar na PSRAM (mais lenta, mas disponível em maior quantidade)
        buf1 = (uint16_t *) heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    }

    buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // Verifique se a alocação falhou (se for muito grande para a SRAM interna)
    if (buf2 == NULL) {
        ESP_LOGE("LVGL", "Falha ao alocar buffer na PSRAM!");
        ESP_LOGI(TAG, "Falhou na alocação de memória no buf2");
        // Se falhar, tenta alocar na PSRAM (mais lenta, mas disponível em maior quantidade)
        buf2 = (uint16_t *) heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    }

    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    //lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    esp_lcd_panel_set_gap(panel_handle, 20, 0);
    //lv_display_set_matrix_rotation(disp, true);

    if (!disp) {
        ESP_LOGE(TAG, "Failed to register display");
        return;
    }

    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_user_data(indev, tp_handle);
    lv_indev_set_read_cb(indev, my_touchpad_read);
    
    assert(heap_caps_check_integrity_all(true));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Start LVGL adapter task
    ESP_ERROR_CHECK(esp_lv_adapter_start());

    // Turn on backlight
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    // Display LVGL demo
    ESP_LOGI(TAG, "Display LVGL demo");
    if ( esp_lv_adapter_lock(-1) == ESP_OK) {
        ESP_LOGI(TAG, "ESP_OK: %d", LV_USE_DEMO_SMARTWATCH);
        //lv_demo_music();
        lv_demo_widgets(); 
        //lv_demo_benchmark();

        //lv_demo_ebike();
        //lv_demo_smartwatch();
        esp_lv_adapter_unlock();
    }
}