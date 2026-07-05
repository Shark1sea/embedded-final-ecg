
#ifndef ECG_AI_DIAGNOSIS_H
#define ECG_AI_DIAGNOSIS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ECG诊断结果结构体
typedef struct {
    bool is_valid;                  // 数据是否有效
    float heart_rate;              // 心率值
    uint8_t rhythm_type;           // 心律类型（0:正常, 1:房颤, 2:早搏, 3:心动过速, 4:心动过缓, 5:未知）
    float confidence;              // 诊断置信度
    char diagnosis_text[64];       // 诊断描述文本
    uint32_t timestamp;            // 诊断时间戳
} ecg_diagnosis_result_t;

// 心律类型枚举
typedef enum {
    RHYTHM_NORMAL = 0,
    RHYTHM_AFIB = 1,
    RHYTHM_PVC = 2,
    RHYTHM_TACHYCARDIA = 3,
    RHYTHM_BRADYCARDIA = 4,
    RHYTHM_UNKNOWN = 5
} ecg_rhythm_type_t;

// AI诊断接口函数
void ecg_ai_init(void);
bool ecg_ai_process_sample(int16_t ecg_sample, ecg_diagnosis_result_t *result);
bool ecg_ai_get_diagnosis(ecg_diagnosis_result_t *result);
void ecg_ai_reset(void);

// 配置接口
void ecg_ai_set_confidence_threshold(float threshold);
void ecg_ai_enable_diagnosis(bool enable);

#ifdef __cplusplus
}
#endif

#endif // ECG_AI_DIAGNOSIS_H
