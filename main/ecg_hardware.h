
#ifndef ECG_HARDWARE_H
#define ECG_HARDWARE_H

#include <stdint.h>
#include <stdbool.h>
#include "ecg_config.h"
#include "esp_adc/adc_continuous.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 硬件状态结构体
 * ============================================================ */
typedef struct {
    bool is_sampling;
    bool is_transmitting;
    bool lon_ok;
    bool lop_ok;
} ecg_hw_state_t;

/* ============================================================
 * 硬件初始化
 * ============================================================ */
void hw_init_all(void);                     // 全部硬件初始化(一站式)
void hw_gpio_init(void);                    // GPIO 初始化
void hw_spi_init(void);                     // SPI 总线初始化
void hw_adc_init(adc_channel_t channel);    // ADC 连续采样初始化
void hw_buzzer_init(void);                  // 蜂鸣器 PWM 初始化
void hw_display_init(void);                 // ILI9488 显示屏初始化
void hw_touch_init(void);                   // XPT2046 触摸初始化
void hw_backlight_init(void);               // 背光 PWM 初始化

/* ============================================================
 * 硬件控制
 * ============================================================ */
void hw_buzzer_on(uint32_t freq_hz);        // 蜂鸣器发声
void hw_buzzer_off(void);                   // 蜂鸣器静音
void hw_backlight_set(uint8_t pct);         // 背光亮度 0-100
void hw_adc_start(void);                    // 启动 ADC 采样
void hw_adc_stop(void);                     // 停止 ADC 采样
void hw_adc_deinit(void);                   // 释放 ADC 资源
void hw_adc_register_done_cb(adc_continuous_evt_cbs_t *cbs); // 注册 ADC 完成回调

/* ============================================================
 * 硬件状态查询
 * ============================================================ */
void hw_read_adc_status(bool *lon_ok, bool *lop_ok);     // 读取导联状态
bool hw_adc_read(uint8_t *buf, uint32_t len, uint32_t *out_len); // 读取 ADC 数据
ecg_hw_state_t* hw_get_state(void);                     // 获取硬件状态
adc_continuous_handle_t hw_get_adc_handle(void);        // 获取 ADC 句柄

/* ============================================================
 * LVGL 集成
 * ============================================================ */
void hw_set_lvgl_disp(lv_disp_drv_t *disp);             // 设置 LVGL 显示驱动
esp_lcd_panel_handle_t hw_get_lcd_panel(void);          // 获取 LCD 面板句柄

/* ============================================================
 * 触摸读取
 * ============================================================ */
bool hw_touch_read(uint16_t *x, uint16_t *y);           // 读取触摸坐标

#ifdef __cplusplus
}
#endif

#endif // ECG_HARDWARE_H
