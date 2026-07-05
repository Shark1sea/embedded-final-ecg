#ifndef ECG_CONFIG_H
#define ECG_CONFIG_H

#include "driver/gpio.h"
#include "hal/adc_types.h"
#include "driver/ledc.h"
#include "soc/soc_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 目标芯片选择
 * ============================================================ */
#if CONFIG_IDF_TARGET_ESP32
    #define ECG_TARGET_ESP32  1
#elif CONFIG_IDF_TARGET_ESP32S3
    #define ECG_TARGET_ESP32S3 1
#else
    #define ECG_TARGET_OTHER   1
#endif

/* ============================================================
 * 引脚定义 (ESP32-S3 & ESP32 通用)
 * ============================================================ */
#if defined(ECG_TARGET_ESP32S3)
    // ESP32-S3 引脚定义
    #define ADC_GPIO                GPIO_NUM_1
    #define ADC_LON_GPIO            GPIO_NUM_5
    #define ADC_LOP_GPIO            GPIO_NUM_6
    #define ADC_CONTROL_GPIO        GPIO_NUM_7

    #define DISPLAY_SPI_CLK         GPIO_NUM_14
    #define DISPLAY_SPI_CS          GPIO_NUM_16
    #define DISPLAY_SPI_MOSI        GPIO_NUM_15
    #define DISPLAY_SPI_MISO        GPIO_NUM_2
    #define DISPLAY_DC              GPIO_NUM_17
    #define DISPLAY_BACKLIGHT       GPIO_NUM_18
    #define DISPLAY_RESET           GPIO_NUM_4

    #define TOUCH_SPI_CLK           GPIO_NUM_12
    #define TOUCH_SPI_CS            GPIO_NUM_21
    #define TOUCH_SPI_MOSI          GPIO_NUM_20
    #define TOUCH_SPI_MISO          GPIO_NUM_19

    #define BUZZER_GPIO             GPIO_NUM_13
    #define ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE2
#elif defined(ECG_TARGET_ESP32)
    // ESP32 引脚定义
    #define ADC_GPIO                GPIO_NUM_34
    #define ADC_LON_GPIO            GPIO_NUM_35
    #define ADC_LOP_GPIO            GPIO_NUM_32
    #define ADC_CONTROL_GPIO        GPIO_NUM_33

    #define DISPLAY_SPI_CLK         GPIO_NUM_14
    #define DISPLAY_SPI_CS          GPIO_NUM_16
    #define DISPLAY_SPI_MOSI        GPIO_NUM_15
    #define DISPLAY_SPI_MISO        GPIO_NUM_2
    #define DISPLAY_DC              GPIO_NUM_17
    #define DISPLAY_BACKLIGHT       GPIO_NUM_18
    #define DISPLAY_RESET           GPIO_NUM_4

    #define TOUCH_SPI_CLK           GPIO_NUM_12
    #define TOUCH_SPI_CS            GPIO_NUM_21
    #define TOUCH_SPI_MOSI          GPIO_NUM_38
    #define TOUCH_SPI_MISO          GPIO_NUM_19

    #define BUZZER_GPIO             GPIO_NUM_13
    #define ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE1
#endif

/* ============================================================
 * ADC 采样配置 —— 降低采样率优化
 *   原: 20kHz → downsample 36 → 555Hz effective
 *   新:  1kHz → downsample  4 → 250Hz effective (满足心电需求)
 * ============================================================ */
#define ADC_UNIT                ADC_UNIT_1
#define ADC_CHANNEL             ADC_CHANNEL_1
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH           SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_SAMPLE_FREQ_HZ      1000        // 从 20kHz 降至 1kHz
#define ADC_READ_LEN            256         // DMA 每次读取字节数
#define ADC_MAX_STORE_BUF_SIZE  4096        // DMA 缓冲区

/* 降采样因子: ADC_SAMPLE_FREQ / DOWNSAMPLE = 250Hz 有效心率 */
#define DOWNSAMPLE_FACTOR       4

/* ============================================================
 * FIR 滤波器配置
 *   Note: 新采样率需要重新设计滤波器系数
 * ============================================================ */
#define FIR_LEN                 64          // FIR 滤波器阶数

/* ============================================================
 * 显示配置 (ILI9488, 480x320)
 * ============================================================ */
#define DISPLAY_H_RES           480
#define DISPLAY_V_RES           320
#define DISPLAY_REFRESH_HZ      (40 * 1000 * 1000)
#define DISPLAY_SPI_QUEUE_LEN   10
#define DISPLAY_CMD_BITS        8
#define DISPLAY_PARAM_BITS      8
#define DISPLAY_MAX_TRANSFER    (DISPLAY_H_RES * 50 * sizeof(uint16_t))
#define LV_BUFFER_SIZE           (DISPLAY_H_RES * 25)

/* ============================================================
 * 触摸配置 (XPT2046)
 * ============================================================ */
#define TOUCH_SPI_FREQ_HZ       (2 * 1000 * 1000)
#define TOUCH_MAX_TRANSFER      4096
#define XPT2046_CMD_X_READ      0x90
#define XPT2046_CMD_Y_READ      0xD0

/* ============================================================
 * 蜂鸣器配置 (LEDC PWM)
 * ============================================================ */
#define BUZZER_LEDC_TIMER       LEDC_TIMER_2
#define BUZZER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_CHANNEL     LEDC_CHANNEL_1
#define BUZZER_LEDC_DUTY_RES    LEDC_TIMER_10_BIT
#define BUZZER_LEDC_FREQ_HZ     2000

/* ============================================================
 * 背光配置 (LEDC PWM)
 * ============================================================ */
#define BACKLIGHT_LEDC_MODE         LEDC_LOW_SPEED_MODE
#define BACKLIGHT_LEDC_CHANNEL      LEDC_CHANNEL_0
#define BACKLIGHT_LEDC_TIMER        LEDC_TIMER_1
#define BACKLIGHT_LEDC_TIMER_RES    LEDC_TIMER_10_BIT
#define BACKLIGHT_LEDC_FREQ_HZ      5000
#define BACKLIGHT_DEFAULT           80
#define BACKLIGHT_MIN               50
#define BACKLIGHT_MAX               100

/* ============================================================
 * LVGL 配置
 * ============================================================ */
#define LVGL_UPDATE_PERIOD_MS   5
#define USE_DOUBLE_BUFFERING    0

/* ============================================================
 * 心率报警默认阈值
 * ============================================================ */
#define HIGH_ALARM_DEFAULT      120     // 心动过速阈值 (bpm)
#define LOW_ALARM_DEFAULT       50      // 心动过缓阈值 (bpm)
#define HIGH_ALARM_MIN          100
#define HIGH_ALARM_MAX          200
#define LOW_ALARM_MIN           30
#define LOW_ALARM_MAX           80

/* ============================================================
 * ECG 波形显示参数
 * ============================================================ */
#define ECG_POINT_COUNT         396     // 显示点数
#define ECG_DISPLAY_HEIGHT      180     // 显示高度
#define ECG_DISPLAY_MIN_Y       1
#define ECG_DISPLAY_MAX_Y       179

/* ============================================================
 * ADC 数据转换宏
 * ============================================================ */
#if ADC_OUTPUT_TYPE == ADC_DIGI_OUTPUT_FORMAT_TYPE2
    // ESP32S3 Type2 格式: 12-bit 数据在低12位
    static inline int16_t adc_raw_to_int16(uint16_t raw) {
        return (int16_t)((int32_t)raw - 2048);  // 12-bit 补码转换
    }
    static inline uint16_t int16_to_adc_raw(int16_t val) {
        return (uint16_t)((int32_t)val + 2048);
    }
    #define ADC_DATA_MIN    0
    #define ADC_DATA_MAX    4095
#else
    // ESP32 Type1 格式
    static inline int16_t adc_raw_to_int16(uint16_t raw) {
        return (int16_t)(raw - 32768);
    }
    static inline uint16_t int16_to_adc_raw(int16_t val) {
        return (uint16_t)(val + 32768);
    }
    #define ADC_DATA_MIN    0
    #define ADC_DATA_MAX    65535
#endif

#ifdef __cplusplus
}
#endif

#endif // ECG_CONFIG_H
