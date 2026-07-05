/**
 * @file    ecg_dsp.c
 * @brief   ECG 数字信号处理: FIR滤波、降采样、Pan-Tompkins QRS检测、心率计算、心律失常检测
 */

#include "ecg_dsp.h"
#include "ecg_config.h"
#include "panTompkins/panTompkins.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include <string.h>
#include <math.h>

static const char *TAG = "DSP";

/* ---- 内部状态 ---- */
static bool     initialized = false;
static float    heart_rate;
static uint32_t qrs_total;
static uint16_t rr_intervals[8];    // 最近 8 个 RR 间期
static uint8_t  rr_idx;
static bool     lead_off;

/* ---- FIR 滤波器 ---- */
static fir_s16_t fir;
static int16_t  fir_input[DOWNSAMPLE_FACTOR];  // 降采样输入缓冲
static uint8_t  fir_input_idx;
static int16_t  fir_output;
static int16_t fir_delay[FIR_LEN];

/* ---- FIR 滤波器系数 (MATLAB 生成) ----
 * 参数: Fs=1000Hz, Fc=40Hz, 64-tap Kaiser (beta=6)
 * 设计工具: MATLAB fir1 + kaiser window
 */
static int16_t fir_coeffs[FIR_LEN] = {
       5,   8,  12,  14,  15,  13,   6,  -7,
     -27, -54, -88,-126,-165,-201,-227,-239,
    -228,-188,-114,   0, 155, 350, 583, 846,
    1131,1424,1713,1983,2220,2411,2545,2614,
    2614,2545,2411,2220,1983,1713,1424,1131,
     846, 583, 350, 155,   0,-114,-188,-228,
    -239,-227,-201,-165,-126, -88, -54, -27,
      -7,   6,  13,  15,  14,  12,   8,   5
};

/* ---- 规则检测阈值 ---- */
#define RR_VARIABILITY_THRESHOLD 0.15f   // RR 变异系数 >15% 视为不齐
#define TACHYCARDIA_THRESHOLD    HIGH_ALARM_DEFAULT
#define BRADYCARDIA_THRESHOLD    LOW_ALARM_DEFAULT
#define PVC_RR_RATIO             0.75f   // 短 RR / 前 RR < 0.75 疑似早搏

/* ============================================================
 * 初始化
 * ============================================================ */
void ecg_dsp_init(void)
{
    memset(fir_input, 0, sizeof(fir_input));
    memset(fir_delay, 0, sizeof(fir_delay));
    memset(rr_intervals, 0, sizeof(rr_intervals));
    fir_input_idx = 0;
    fir_output    = 0;
    heart_rate    = 0.0f;
    qrs_total     = 0;
    rr_idx        = 0;
    lead_off      = false;

    // 使用 esp-dsp 初始化 FIR
    dsps_fird_init_s16(&fir, fir_coeffs, fir_delay, FIR_LEN,
                       DOWNSAMPLE_FACTOR, 0, 0);

    panTompkins_init();
    initialized = true;
    ESP_LOGI(TAG, "DSP 初始化完成 (Fs=%d Hz, downsample=%d, effective=%d Hz)",
             ADC_SAMPLE_FREQ_HZ, DOWNSAMPLE_FACTOR,
             ADC_SAMPLE_FREQ_HZ / DOWNSAMPLE_FACTOR);
}

/* ============================================================
 * 处理原始 ADC 数据
 * ============================================================ */
bool ecg_dsp_process_raw(uint16_t raw_adc, ecg_dsp_result_t *result)
{
    if (!initialized) return false;

    if (result) memset(result, 0, sizeof(*result));

    // 1. 转换为有符号 int16
    fir_input[fir_input_idx++] = adc_raw_to_int16(raw_adc);

    // 2. 降采样 + FIR 滤波
    if (fir_input_idx >= DOWNSAMPLE_FACTOR) {
        fir_input_idx = 0;

        int32_t actual;
#if CONFIG_IDF_TARGET_ESP32
        actual = dsps_fird_s16_ae32(&fir, fir_input, &fir_output, 1);
#elif CONFIG_IDF_TARGET_ESP32S3
        actual = dsps_fird_s16_aes3(&fir, fir_input, &fir_output, 1);
#else
        actual = dsps_fird_s16_ansi(&fir, fir_input, &fir_output, 1);
#endif

        if (actual <= 0) {
            // 滤波未产生输出（处于启动阶段）
            return false;
        }

        // 3. 送入 Pan-Tompkins 算法
        uint16_t pt_input = int16_to_adc_raw(fir_output);
        panTompkins_process((dataType)pt_input);

        // 4. 检查 QRS 检测
        uint32_t detected_idx;
        if (panTompkins_get_detection(&detected_idx)) {
            heart_rate = panTompkins_get_heart_rate();
            qrs_total++;

            // 记录 RR 间期（用于心律失常检测）
            // 从心率反算: RR(ms) = 60000 / HR
            if (heart_rate > 0) {
                rr_intervals[rr_idx] = (uint16_t)(60000.0f / heart_rate);
                rr_idx = (rr_idx + 1) % 8;
            }
        }

        // 填充结果
        if (result) {
            result->filtered_sample = fir_output;
            result->heart_rate      = heart_rate;
            result->qrs_count       = qrs_total;
            result->lead_off        = lead_off;
        }

        return true;
    }

    return false;
}

/* ============================================================
 * 心率获取
 * ============================================================ */
float ecg_dsp_get_heart_rate(void)
{
    return heart_rate;
}

/* ============================================================
 * 基于规则的心律失常检测
 * ============================================================ */
ecg_arrhythmia_t ecg_dsp_detect_arrhythmia(void)
{
    if (qrs_total < 5) return ARRHYTHMIA_NONE;

    // 心动过速
    if (heart_rate > TACHYCARDIA_THRESHOLD)
        return ARRHYTHMIA_TACHYCARDIA;

    // 心动过缓
    if (heart_rate < BRADYCARDIA_THRESHOLD && heart_rate > 0)
        return ARRHYTHMIA_BRADYCARDIA;

    // RR 间期分析 (需要至少 3 个 RR)
    int valid_rr = 0;
    float rr_sum = 0;
    for (int i = 0; i < 8; i++) {
        if (rr_intervals[i] > 0) {
            rr_sum += rr_intervals[i];
            valid_rr++;
        }
    }

    if (valid_rr >= 3) {
        float rr_mean = rr_sum / valid_rr;

        // 计算变异系数 CV
        float variance = 0;
        for (int i = 0; i < 8; i++) {
            if (rr_intervals[i] > 0) {
                float d = rr_intervals[i] - rr_mean;
                variance += d * d;
            }
        }
        variance /= valid_rr;
        float cv = sqrtf(variance) / rr_mean;

        // 房颤: RR 间期绝对不齐 (CV > 阈值)
        if (cv > RR_VARIABILITY_THRESHOLD * 2.0f)
            return ARRHYTHMIA_AFIB_SUSPECT;

        // 心律不齐
        if (cv > RR_VARIABILITY_THRESHOLD)
            return ARRHYTHMIA_IRREGULAR;
    }

    return ARRHYTHMIA_NONE;
}

/* ============================================================
 * RR 间期获取
 * ============================================================ */
int ecg_dsp_get_rr_intervals(uint16_t *buf, int max_count)
{
    int count = 0;
    for (int i = 0; i < max_count && i < 8; i++) {
        int idx = (rr_idx - 1 - i + 8) % 8; // 从最新开始
        if (rr_intervals[idx] > 0) {
            buf[count++] = rr_intervals[idx];
        }
    }
    return count;
}

/* ============================================================
 * 重置
 * ============================================================ */
void ecg_dsp_reset(void)
{
    memset(fir_input, 0, sizeof(fir_input));
    memset(fir_delay, 0, sizeof(fir_delay));
    memset(rr_intervals, 0, sizeof(rr_intervals));
    fir_input_idx = 0;
    fir_output    = 0;
    heart_rate    = 0.0f;
    qrs_total     = 0;
    rr_idx        = 0;
    panTompkins_init();
}
