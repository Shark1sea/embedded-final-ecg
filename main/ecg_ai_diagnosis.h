
#ifndef ECG_AI_DIAGNOSIS_H
#define ECG_AI_DIAGNOSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "ecg_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 诊断结果
 * ============================================================ */
typedef enum {
    DIAG_NORMAL          = 0,   // 正常窦性心律
    DIAG_TACHYCARDIA     = 1,   // 心动过速
    DIAG_BRADYCARDIA     = 2,   // 心动过缓
    DIAG_IRREGULAR       = 3,   // 心律不齐
    DIAG_AFIB_SUSPECT    = 4,   // 疑似房颤
    DIAG_PVC_SUSPECT     = 5,   // 疑似早搏
    DIAG_NOISE           = 6,   // 信号噪声过大
    DIAG_LEAD_OFF        = 7,   // 导联脱落
    DIAG_UNKNOWN         = 99,  // 未知
} ecg_diagnosis_t;

typedef struct {
    ecg_diagnosis_t diagnosis;          // 诊断类型
    float           confidence;         // 置信度 0-1
    float           heart_rate;         // 心率
    char            text[64];           // 可读诊断文本
    uint32_t        timestamp;          // 时间戳
    bool            is_valid;           // 结果是否有效
} ecg_diag_result_t;

/* ============================================================
 * AI 诊断配置
 * ============================================================ */
typedef struct {
    float confidence_threshold;         // 置信度阈值 (默认 0.6)
    bool  enable_rule_based;            // 启用规则诊断
    bool  enable_nn_inference;          // 启用神经网络推理 (需模型)
    int   analysis_window_sec;          // 分析窗口 (秒)
} ecg_ai_config_t;

/* ============================================================
 * API
 * ============================================================ */
void ecg_ai_init(void);
bool ecg_ai_process(const ecg_dsp_result_t *dsp, ecg_diag_result_t *result);
const ecg_diag_result_t* ecg_ai_get_last(void);
void ecg_ai_set_config(const ecg_ai_config_t *cfg);
void ecg_ai_reset(void);

#ifdef __cplusplus
}
#endif

#endif // ECG_AI_DIAGNOSIS_H
