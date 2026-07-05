/**
 * @file    ecg_ai_diagnosis.c
 * @brief   ECG AI 诊断模块: 基于规则 + 预留神经网络推理接口
 * @note    当前实现基于规则的诊断，ESP32S3 NN 推理接口已预留
 *          未来可集成 ESP-DL 或 TensorFlow Lite Micro 模型
 */

#include "ecg_ai_diagnosis.h"
#include "ecg_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "AI";

/* ---- 内部状态 ---- */
static ecg_ai_config_t config = {
    .confidence_threshold = 0.6f,
    .enable_rule_based   = true,
    .enable_nn_inference  = false,  // 需要部署模型后启用
    .analysis_window_sec  = 5,
};

static ecg_diag_result_t last_diag;
static bool initialized;

/* ---- 诊断文本表 ---- */
static const char *diag_texts[] = {
    [DIAG_NORMAL]       = "正常窦性心律",
    [DIAG_TACHYCARDIA]  = "心动过速",
    [DIAG_BRADYCARDIA]  = "心动过缓",
    [DIAG_IRREGULAR]    = "心律不齐",
    [DIAG_AFIB_SUSPECT] = "疑似房颤",
    [DIAG_PVC_SUSPECT]  = "疑似早搏",
    [DIAG_NOISE]        = "信号噪声过大",
    [DIAG_LEAD_OFF]     = "导联脱落",
    [DIAG_UNKNOWN]      = "未知",
};

/* ---- 滑动窗口统计 ---- */
#define HISTORY_LEN 32
static ecg_diagnosis_t diag_history[HISTORY_LEN];
static uint8_t        diag_hist_idx;
static uint32_t       diag_hist_count;

/* ============================================================
 * 初始化
 * ============================================================ */
void ecg_ai_init(void)
{
    memset(&last_diag, 0, sizeof(last_diag));
    memset(diag_history, 0, sizeof(diag_history));
    diag_hist_idx   = 0;
    diag_hist_count = 0;
    initialized     = true;
    ESP_LOGI(TAG, "AI 诊断模块初始化完成 (规则引擎: %s, NN推理: %s)",
             config.enable_rule_based ? "ON" : "OFF",
             config.enable_nn_inference  ? "ON" : "OFF");
}

/* ============================================================
 * 处理
 * ============================================================ */
bool ecg_ai_process(const ecg_dsp_result_t *dsp, ecg_diag_result_t *result)
{
    if (!initialized || !dsp) return false;

    ecg_diag_result_t diag = {0};
    diag.timestamp = esp_timer_get_time() / 1000;
    diag.heart_rate = dsp->heart_rate;

    // 导联脱落检查
    if (dsp->lead_off) {
        diag.diagnosis  = DIAG_LEAD_OFF;
        diag.confidence = 0.95f;
        diag.is_valid   = true;
        snprintf(diag.text, sizeof(diag.text), "导联脱落，请检查电极连接");
        goto done;
    }

    // 规则诊断
    if (config.enable_rule_based) {
        ecg_arrhythmia_t arr = ecg_dsp_detect_arrhythmia();

        switch (arr) {
        case ARRHYTHMIA_TACHYCARDIA:
            diag.diagnosis  = DIAG_TACHYCARDIA;
            diag.confidence = 0.85f;
            snprintf(diag.text, sizeof(diag.text), "心动过速: %.0f bpm", dsp->heart_rate);
            break;
        case ARRHYTHMIA_BRADYCARDIA:
            diag.diagnosis  = DIAG_BRADYCARDIA;
            diag.confidence = 0.85f;
            snprintf(diag.text, sizeof(diag.text), "心动过缓: %.0f bpm", dsp->heart_rate);
            break;
        case ARRHYTHMIA_AFIB_SUSPECT:
            diag.diagnosis  = DIAG_AFIB_SUSPECT;
            diag.confidence = 0.55f;  // 规则检测准确度有限
            snprintf(diag.text, sizeof(diag.text), "疑似房颤，建议进一步检查");
            break;
        case ARRHYTHMIA_IRREGULAR:
            diag.diagnosis  = DIAG_IRREGULAR;
            diag.confidence = 0.65f;
            snprintf(diag.text, sizeof(diag.text), "心律不齐: %.0f bpm", dsp->heart_rate);
            break;
        default:
            // 信号质量评估
            if (dsp->signal_clipped) {
                diag.diagnosis  = DIAG_NOISE;
                diag.confidence = 0.70f;
                snprintf(diag.text, sizeof(diag.text), "信号质量差，请调整电极位置");
            } else if (dsp->heart_rate > 0) {
                diag.diagnosis  = DIAG_NORMAL;
                diag.confidence = 0.80f;
                snprintf(diag.text, sizeof(diag.text), "正常窦性心律: %.0f bpm", dsp->heart_rate);
            }
            break;
        }
        diag.is_valid = true;
    }

done:
    // 记录历史
    diag_history[diag_hist_idx] = diag.diagnosis;
    diag_hist_idx = (diag_hist_idx + 1) % HISTORY_LEN;
    diag_hist_count++;

    // 持续诊断平滑: 需要连续 N 次相同诊断才确认
    if (diag_hist_count >= 3) {
        uint8_t same_count = 1;
        for (int i = 1; i < 3; i++) {
            uint8_t prev = (diag_hist_idx - i + HISTORY_LEN) % HISTORY_LEN;
            if (diag_history[prev] == diag.diagnosis) same_count++;
        }
        if (same_count >= 3) {
            last_diag = diag; // 确认诊断
        }
    } else {
        last_diag = diag;
    }

    if (result) *result = last_diag;

    // 只在诊断变更时返回 true
    return diag.is_valid && (diag.diagnosis != last_diag.diagnosis || diag_hist_count <= 3);
}

/* ============================================================
 * 获取最近结果
 * ============================================================ */
const ecg_diag_result_t* ecg_ai_get_last(void)
{
    return &last_diag;
}

/* ============================================================
 * 配置
 * ============================================================ */
void ecg_ai_set_config(const ecg_ai_config_t *cfg)
{
    if (cfg) config = *cfg;
}

/* ============================================================
 * 重置
 * ============================================================ */
void ecg_ai_reset(void)
{
    memset(&last_diag, 0, sizeof(last_diag));
    memset(diag_history, 0, sizeof(diag_history));
    diag_hist_idx   = 0;
    diag_hist_count = 0;
}

/* ============================================================
 * NN 推理预留接口 (ESP32S3)
 * ============================================================
 * 使用方式:
 *   1. 使用 TensorFlow Lite Micro 或 ESP-DL 训练模型
 *   2. 将 .tflite/.cpp 模型放入 components/ecg_model/
 *   3. 设置 config.enable_nn_inference = true
 *   4. 在下面实现具体的推理调用
 *
 * 示例框架:
 *   #include "tensorflow/lite/micro/micro_interpreter.h"
 *   static tflite::MicroInterpreter *interpreter = NULL;
 *   ...
 */
