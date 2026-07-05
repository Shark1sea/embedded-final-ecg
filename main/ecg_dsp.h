#ifndef ECG_DSP_H
#define ECG_DSP_H

#include <stdint.h>
#include <stdbool.h>
#include "ecg_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * DSP 处理结果
 * ============================================================ */
typedef struct {
    float    heart_rate;         // 当前心率 (bpm)
    bool     qrs_detected;       // 是否检测到 QRS 波
    uint32_t qrs_count;          // QRS 累计计数
    int16_t  filtered_sample;    // 滤波后采样值
    bool     lead_off;           // 导联脱落
    bool     signal_clipped;     // 信号削顶
    float    snr_estimate;       // 信噪比估计 (dB)
} ecg_dsp_result_t;

/* ============================================================
 * 心律失常类型 (基于规则)
 * ============================================================ */
typedef enum {
    ARRHYTHMIA_NONE = 0,
    ARRHYTHMIA_TACHYCARDIA,     // 心动过速
    ARRHYTHMIA_BRADYCARDIA,     // 心动过缓
    ARRHYTHMIA_IRREGULAR,       // 心律不齐 (RR间期变异大)
    ARRHYTHMIA_PVC_SUSPECT,     // 疑似早搏 (宽QRS)
    ARRHYTHMIA_AFIB_SUSPECT,    // 疑似房颤 (RR间期绝对不齐)
} ecg_arrhythmia_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化 DSP 模块 (FIR 滤波器 + Pan-Tompkins)
 */
void ecg_dsp_init(void);

/**
 * @brief 处理一个原始 ADC 采样值 (uint16)
 * @param raw_adc    ADC 原始值
 * @param result     输出处理结果 (可为 NULL)
 * @return           是否有新的滤波输出
 */
bool ecg_dsp_process_raw(uint16_t raw_adc, ecg_dsp_result_t *result);

/**
 * @brief 获取当前心率
 */
float ecg_dsp_get_heart_rate(void);

/**
 * @brief 基于规则的心律失常检测
 * @return 心律失常类型
 */
ecg_arrhythmia_t ecg_dsp_detect_arrhythmia(void);

/**
 * @brief 获取最近 N 个 RR 间期 (ms)
 * @param rr_buf     输出缓冲区
 * @param max_count  最大数量
 * @return           实际填充数量
 */
int ecg_dsp_get_rr_intervals(uint16_t *rr_buf, int max_count);

/**
 * @brief 重置 DSP 状态
 */
void ecg_dsp_reset(void);

#ifdef __cplusplus
}
#endif

#endif // ECG_DSP_H
