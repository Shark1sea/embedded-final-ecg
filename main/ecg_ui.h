#ifndef ECG_UI_H
#define ECG_UI_H

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"
#include "ecg_config.h"
#include "ecg_dsp.h"
#include "ecg_ai_diagnosis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * UI 回调函数指针 (由 main.c 注册)
 * ============================================================ */
typedef void (*ui_ecg_start_cb_t)(void);
typedef void (*ui_ecg_stop_cb_t)(void);
typedef void (*ui_transmit_start_cb_t)(void);
typedef void (*ui_transmit_stop_cb_t)(void);
typedef void (*ui_backlight_set_cb_t)(uint8_t pct);
typedef void (*ui_alarm_high_set_cb_t)(uint16_t bpm);
typedef void (*ui_alarm_low_set_cb_t)(uint16_t bpm);

/* ============================================================
 * UI 初始化
 * ============================================================ */
void ui_init(void);         // LVGL 图形库 + 触摸 + 样式 + 界面
uint32_t ui_tick(void);    // 每帧调用 lv_timer_handler，返回下次等待 ms
void ui_flush(void);        // 将共享缓冲刷新到 LVGL (由 ui_tick 内部调用)

/* ============================================================
 * 数据更新接口 (跨核安全: Core 1 只写共享缓冲, Core 0 刷新)
 * ============================================================ */
void ui_update_heart_rate(float bpm);                   // 更新心率显示
void ui_update_waveform(const int16_t *data, uint16_t count); // 更新波形
void ui_update_warning(const char *text, bool is_warn); // 更新警告标签
void ui_update_sampling_rate_label(void);               // 刷新采样率标签
void ui_update_diagnosis(const ecg_diag_result_t *d);   // 更新 AI 诊断
void ui_set_sampling_indicator(bool active);            // 显示/隐藏刷新指示器
void ui_reset_waveform(void);                           // 清空波形

/* ============================================================
 * 回调注册
 * ============================================================ */
void ui_set_callbacks(
    ui_ecg_start_cb_t      ecg_start,
    ui_ecg_stop_cb_t       ecg_stop,
    ui_transmit_start_cb_t tx_start,
    ui_transmit_stop_cb_t  tx_stop,
    ui_backlight_set_cb_t  bl_set,
    ui_alarm_high_set_cb_t alarm_high,
    ui_alarm_low_set_cb_t  alarm_low
);

/* ============================================================
 * 状态查询
 * ============================================================ */
bool ui_is_ecg_active(void);        // 是否按下开始采集
bool ui_is_transmit_active(void);   // 是否按下串口传输
uint8_t ui_get_backlight(void);     // 获取背光设定值
uint16_t ui_get_alarm_high(void);   // 获取高心率阈值
uint16_t ui_get_alarm_low(void);    // 获取低心率阈值

#ifdef __cplusplus
}
#endif

#endif // ECG_UI_H
