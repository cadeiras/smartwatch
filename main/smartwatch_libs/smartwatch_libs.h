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
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  (1)

// LCD resolution configuration
#define EXAMPLE_LCD_H_RES              (280)
#define EXAMPLE_LCD_V_RES              (456)

// Color depth configuration
#define LCD_BIT_PER_PIXEL              (16)

#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 4)

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

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 * GLOBAL PROTOTYPES
 **********************/


/**
 * Document here!
 */
static void smartwatch_touch_read(lv_indev_t * indev, lv_indev_data_t * data);

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
