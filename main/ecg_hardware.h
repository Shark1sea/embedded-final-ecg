
#ifndef ECG_HARDWARE_H
#define ECG_HARDWARE_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============= 引脚定义 =============
#define ADC_GPIO_NUM                GPIO_NUM_1  // ESP32-S3 可用ADC引脚
#define ADC_LON_GPIO_NUM            GPIO_NUM_2
#define ADC_LOP_GPIO_NUM            GPIO_NUM_3
#define ADC_CONTROL_GPIO_NUM        GPIO_NUM_4

#define DISPLAY_SPI_CLK             GPIO_NUM_14
#define DISPLAY_SPI_CS              GPIO_NUM_16
#define DISPLAY_SPI_MOSI            GPIO_NUM_15
#define DISPLAY_SPI_MISO            GPIO_NUM_2
#define DISPLAY_DC                  GPIO_NUM_17
#define DISPLAY_BACKLIGHT           GPIO_NUM_18
#define DISPLAY_RESET               GPIO_NUM_5

#define TOUCH_SPI_CLK               GPIO_NUM_12
#define TOUCH_SPI_CS                GPIO_NUM_21
#define TOUCH_SPI_MOSI              GPIO_NUM_38
#define TOUCH_SPI_MISO              GPIO_NUM_19

#define BUZZER_GPIO_NUM             GPIO_NUM_13

// ============= ADC配置 =============
#define ADC_UNIT                    ADC_UNIT_1
#define ADC_ATTEN                   ADC_ATTEN_DB_12
#define ADC_SAMPLE_FREQ_HZ          20000
#define ADC_READ_LEN                256

// ============= 显示配置 =============
#define DISPLAY_H_RES               480
#define DISPLAY_V_RES               320
#define DISPLAY_REFRESH_HZ          40000000
#define LV_BUFFER_SIZE              (DISPLAY_H_RES * 25)

// ============= 蜂鸣器配置 =============
#define BUZZER_LEDC_TIMER           LEDC_TIMER_2
#define BUZZER_LEDC_MODE            LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_CHANNEL         LEDC_CHANNEL_1
#define BUZZER_LEDC_DUTY_RES        LEDC_TIMER_10_BIT
#define BUZZER_LEDC_FREQ_HZ         2000

// ============= 背光配置 =============
#define BACKLIGHT_LEDC_MODE         LEDC_LOW_SPEED_MODE
#define BACKLIGHT_LEDC_CHANNEL      LEDC_CHANNEL_0
#define BACKLIGHT_LEDC_TIMER        LEDC_TIMER_1
#define BACKLIGHT_LEDC_TIMER_RES    LEDC_TIMER_10_BIT
#define BACKLIGHT_LEDC_FREQ_HZ      5000
#define BACKLIGHT_DEFAULT           80
#define BACKLIGHT_MIN               50
#define BACKLIGHT_MAX               100

// ============= ECG状态 =============
typedef struct {
    bool is_sampling;
    bool is_transmitting;
} ecg_hw_state_t;

// ============= 硬件初始化函数 =============
void hw_gpio_init(void);
void hw_spi_init(void);
void hw_adc_init(void);
void hw_buzzer_init(void);
void hw_display_init(void);
void hw_touch_init(void);
void hw_backlight_init(void);

// ============= 硬件控制函数 =============
void hw_buzzer_on(uint32_t freq_hz);
void hw_buzzer_off(void);
void hw_backlight_set(uint8_t brightness);
void hw_adc_start(void);
void hw_adc_stop(void);
void hw_read_adc_status(bool *lon_ok, bool *lop_ok);

// ============= 数据获取函数 =============
bool hw_adc_read(uint8_t *buffer, uint32_t buf_size, uint32_t *out_size);

// ============= 状态获取 =============
ecg_hw_state_t* hw_get_state(void);

// ============= 显示屏句柄获取 =============
void* hw_get_lcd_panel_handle(void);

// ============= 设置 LVGL display 句柄（用于 flush 完成通知） =============
// 必须在 lv_display_create 之后调用
typedef struct _lv_display_t lv_display_t;
void hw_set_lvgl_display(lv_display_t *disp);

// ============= 调试函数：显示简单测试图案 =============
void hw_test_display_pattern(void);

#ifdef __cplusplus
}
#endif

#endif // ECG_HARDWARE_H
