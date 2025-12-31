/**
 * @file smartwatch_libs.h
 *
 */

#ifndef SMARTWATCH_LIBS_H
#define SMARTWATCH_LIBS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/
#define TEST_I2C_PORT I2C_NUM_0

#define ESP_LCD_TOUCH_IO_I2C_FT3168_ADDRESS 0x38

#define I2C_MASTER_SCL_IO 48
#define I2C_MASTER_SDA_IO 47

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// LCD Configuration ////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_HOST               (SPI2_HOST)
#define EXAMPLE_PIN_NUM_LCD_CS         (GPIO_NUM_9)
#define EXAMPLE_PIN_NUM_LCD_PCLK       (GPIO_NUM_10)
#define EXAMPLE_PIN_NUM_LCD_DATA0      (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_LCD_DATA1      (GPIO_NUM_12)
#define EXAMPLE_PIN_NUM_LCD_DATA2      (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_LCD_DATA3      (GPIO_NUM_14)
#define EXAMPLE_PIN_NUM_LCD_RST        (GPIO_NUM_21)
#define EXAMPLE_PIN_NUM_BK_LIGHT       (GPIO_NUM_1)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  (-1)

// LCD resolution configuration
#define EXAMPLE_LCD_H_RES              (280)
#define EXAMPLE_LCD_V_RES              (456)

// Color depth configuration
#define LCD_BIT_PER_PIXEL              (16)

#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 5)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Touch Configuration //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
#define EXAMPLE_TOUCH_HOST             (I2C_NUM_0)
#define EXAMPLE_PIN_NUM_TOUCH_SCL      (GPIO_NUM_48)
#define EXAMPLE_PIN_NUM_TOUCH_SDA      (GPIO_NUM_47)
#define EXAMPLE_PIN_NUM_TOUCH_RST      (GPIO_NUM_21)  // Not used
#define EXAMPLE_PIN_NUM_TOUCH_INT      (GPIO_NUM_5)
//#endif

#define ESP_LCD_TOUCH_IO_I2C_FT3168_CONFIG()            \ 
{                                       \
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS, \
        .control_phase_bytes = 1,           \
        .dc_bit_offset = 0,                 \
        .lcd_cmd_bits = 8,                  \
        .lcd_param_bits = 0,                              \
        .flags =                            \
        {                                   \
            .disable_control_phase = 1,     \
        },                                   \
        .scl_speed_hz = 400000                            \
    }
/*
#define ESP_LCD_TOUCH_IO_I2C_FT3168_CONFIG()            \ 
{                                                         \
    .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT3168_ADDRESS,  \
    .control_phase_bytes = 1,                         \
    .dc_bit_offset = 0,                               \
    .lcd_cmd_bits = 8,                                \
    .lcd_param_bits = 0,                              \
    .flags =                                          \
    {                                                 \
       .dc_low_on_data = 0,                          \
       .disable_control_phase = 1,                   \
    },                                                \
    .scl_speed_hz = 400000                            \
}
    */

#define SH8601_PANEL_IO_QSPI_CONFIG(cs, cb, cb_ctx)             \
{                                                           \
    .cs_gpio_num = cs,                                      \
    .dc_gpio_num = -1,                                      \
    .spi_mode = 0,                                          \
    .pclk_hz = 40 * 1000 * 1000,                            \
    .trans_queue_depth = 10,                                \
    .user_ctx = cb_ctx,                                     \
    .lcd_cmd_bits = 32,                                     \
    .lcd_param_bits = 8,                                    \
    .flags = {                                              \
        .quad_mode = true,                                  \
    },                                                      \
}

/**
 * @brief Default configuration values for LVGL adapter
 * @{
 */
#define EBIKE_LV_ADAPTER_DEFAULT_STACK_SIZE        (12 * 1024)  /*!< Default task stack size (8KB) */
#define EBIKE_LV_ADAPTER_DEFAULT_TASK_PRIORITY     2           /*!< Default task priority */
#define EBIKE_LV_ADAPTER_DEFAULT_TASK_CORE_ID      (-1)        /*!< Default task core ID (-1 for no affinity) */
#define EBIKE_LV_ADAPTER_DEFAULT_TICK_PERIOD_MS    2           /*!< Default tick period in milliseconds */
#define EBIKE_LV_ADAPTER_DEFAULT_TASK_MIN_DELAY_MS 1           /*!< Default minimum task delay in milliseconds */
#define EBIKE_LV_ADAPTER_DEFAULT_TASK_MAX_DELAY_MS 500          /*!< Default maximum task delay in milliseconds */
/** @} */

/**
 * @brief Default configuration for LVGL adapter
 */
#define EBIKE_LV_ADAPTER_DEFAULT_CONFIG() {                            \
    .task_stack_size   = EBIKE_LV_ADAPTER_DEFAULT_STACK_SIZE,          \
    .task_priority     = EBIKE_LV_ADAPTER_DEFAULT_TASK_PRIORITY,       \
    .task_core_id      = EBIKE_LV_ADAPTER_DEFAULT_TASK_CORE_ID,        \
    .tick_period_ms    = EBIKE_LV_ADAPTER_DEFAULT_TICK_PERIOD_MS,      \
    .task_min_delay_ms = EBIKE_LV_ADAPTER_DEFAULT_TASK_MIN_DELAY_MS,   \
    .task_max_delay_ms = EBIKE_LV_ADAPTER_DEFAULT_TASK_MAX_DELAY_MS,   \
    .stack_in_psram    = true,                                      \
}

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/


/**
 * Document here!
 */
bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);

/**
 * Document here!
 */
void my_touchpad_read(lv_indev_t * indev, lv_indev_data_t * data);



/**********************
 * GLOBAL VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/



#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*SMARTWATCH_LIBS_H*/
