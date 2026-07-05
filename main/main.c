/**
 * @file    main.c
 * @brief   ECG 智能心电监测系统 - 主入口
 * @note    模块化架构: config → hardware → dsp → ai → ui → main
 */

#include "ecg_config.h"
#include "ecg_hardware.h"
#include "ecg_dsp.h"
#include "ecg_ai_diagnosis.h"
#include "ecg_ui.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---- ADC 数据提取宏 (ESP-IDF v6.0.1 需手动定义) ---- */
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#define ADC_GET_CHANNEL(p)  ((p)->type2.channel)
#define ADC_GET_DATA(p)     ((p)->type2.data)
#else
#define ADC_GET_CHANNEL(p)  ((p)->type1.channel)
#define ADC_GET_DATA(p)     ((p)->type1.data)
#endif

static const char *TAG = "MAIN";

/* ---- 任务句柄 ---- */
static TaskHandle_t task_ecg      = NULL;

/* ---- 状态 ---- */
static volatile bool sampling_active = false;
static float          current_hr     = 0.0f;

/* ---- 阈值 ---- */
static uint16_t alarm_high = HIGH_ALARM_DEFAULT;
static uint16_t alarm_low  = LOW_ALARM_DEFAULT;

/* ---- 标签状态追踪: 只在状态变化时更新 UI, 避免高频 spinlock ---- */
static int  prev_abnormal = -1;   // -1=初始, 0=正常, 1=异常
static uint32_t last_warn_tick = 0;
static uint32_t last_ai_tick   = 0;

/* ============================================================
 * 预留: 警报动作接口 (未来可接入 BLE/WiFi 通知)
 * ============================================================ */
typedef void (*alarm_action_cb_t)(float bpm, bool is_abnormal);
static alarm_action_cb_t alarm_action_cb = NULL;

void ecg_register_alarm_action(alarm_action_cb_t cb)
{
    alarm_action_cb = cb;
}

/* ============================================================
 * UI 回调 (UI → main)
 * ============================================================ */
static void on_ecg_start(void)
{
    sampling_active = true;
    ecg_dsp_reset();
    ecg_ai_reset();
    ui_update_warning("采集启动中...", false);
}

static void on_ecg_stop(void)
{
    sampling_active = false;
    ui_update_warning("停止采集...", false);
}

static void on_tx_start(void) { /* TODO: 串口传输 */ }
static void on_tx_stop(void)  { /* TODO: 串口传输 */ }

static void on_backlight_set(uint8_t pct)
{
    hw_backlight_set(pct);
}

static void on_alarm_high_set(uint16_t bpm) { alarm_high = bpm; }
static void on_alarm_low_set(uint16_t bpm)  { alarm_low  = bpm; }

/* ============================================================
 * ECG 采样任务 (核心)
 * ============================================================ */
static void ecg_task(void *pv)
{
    esp_task_wdt_add(NULL);

    // 初始化 ADC
    hw_adc_init(ADC_CHANNEL);

    // 初始化 DSP & AI
    ecg_dsp_init();
    ecg_ai_init();

    // 显示指示器
    ui_set_sampling_indicator(true);
    ui_reset_waveform();

    // 启动 ADC
    hw_adc_start();

    // 降采样缓冲（每3个滤波输出画一个点，~75Hz刷新）
    int16_t filtered_buf[3];
    uint8_t filtered_idx = 0;

    ESP_LOGI(TAG, "ECG 采样任务启动 (Fs=%d Hz, effective=%d Hz)",
             ADC_SAMPLE_FREQ_HZ, ADC_SAMPLE_FREQ_HZ / DOWNSAMPLE_FACTOR);

    while (sampling_active) {
        esp_task_wdt_reset();

        // 读取 ADC
        uint8_t buf[ADC_READ_LEN];
        uint32_t len;
        if (!hw_adc_read(buf, sizeof(buf), &len)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // 解析 ADC 数据
        for (uint32_t i = 0; i < len; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&buf[i];
            uint16_t chan = ADC_GET_CHANNEL(p);
            if (chan >= SOC_ADC_CHANNEL_NUM(ADC_UNIT)) continue;

            uint16_t raw = ADC_GET_DATA(p);

            // DSP 处理
            ecg_dsp_result_t dsp_res;
            if (ecg_dsp_process_raw(raw, &dsp_res)) {
                // 持续喂 AI 状态机 (轻量级调用)
                ecg_ai_process(&dsp_res, NULL);

                // 心率更新
                if (dsp_res.heart_rate > 0) {
                    current_hr = dsp_res.heart_rate;
                    if (dsp_res.qrs_count >= 5) {
                        ui_update_heart_rate(current_hr);
                    }
                }

                // 波形数据收集
                filtered_buf[filtered_idx++] = dsp_res.filtered_sample;
                if (filtered_idx >= 3) {
                    ui_update_waveform(&filtered_buf[1], 1); // 中值
                    filtered_idx = 0;
                }
            }
        }

        // --- 以下逻辑每轮只执行一次, 避免在 DSP 内循环中高频调用 ---

        // 标签心率提示 & 预留警报: 状态变化时更新
        if (current_hr > 0) {
            int abnormal = (current_hr > alarm_high || current_hr < alarm_low) ? 1 : 0;
            uint32_t now = xTaskGetTickCount();
            if (abnormal != prev_abnormal || (now - last_warn_tick >= pdMS_TO_TICKS(5000))) {
                last_warn_tick = now;
                prev_abnormal = abnormal;
                if (abnormal) {
                    ui_update_warning("心率异常", true);
                } else {
                    ui_update_warning("心率正常", false);
                }
                // 预留: BLE/WiFi 警报
                if (alarm_action_cb) {
                    alarm_action_cb(current_hr, abnormal);
                }
            }

            // AI 诊断 (每 5s 输出一次, 同步更新 UI 标签)
            if (now - last_ai_tick >= pdMS_TO_TICKS(5000)) {
                last_ai_tick = now;
                const ecg_diag_result_t *ai_res = ecg_ai_get_last();
                if (ai_res->is_valid) {
                    ESP_LOGI(TAG, "AI: %s (置信度: %.2f)",
                             ai_res->text, (double)ai_res->confidence);
                    ui_update_diagnosis(ai_res);  // 串连 UI 标签
                }
            }
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 停止并清理
    hw_adc_stop();
    hw_adc_deinit();
    ui_set_sampling_indicator(false);
    ui_update_heart_rate(0);
    current_hr = 0.0f;
    ESP_LOGI(TAG, "ECG 采样任务已停止");

    esp_task_wdt_delete(NULL);
    task_ecg = NULL;
    vTaskDelete(NULL);
}

/* ============================================================
 * 主入口
 * ============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== ECG 智能心电监测系统 ===");
    ESP_LOGI(TAG, "芯片: %s | ADC: %d Hz | 有效采样: %d Hz",
             CONFIG_IDF_TARGET, ADC_SAMPLE_FREQ_HZ,
             ADC_SAMPLE_FREQ_HZ / DOWNSAMPLE_FACTOR);

    /* ---- 硬件初始化 ---- */
    hw_init_all();

    /* ---- UI 初始化 ---- */
    ui_init();
    ui_set_callbacks(on_ecg_start, on_ecg_stop,
                     on_tx_start, on_tx_stop,
                     on_backlight_set, on_alarm_high_set, on_alarm_low_set);
    hw_backlight_set(BACKLIGHT_DEFAULT);

    /* ---- 主循环 ---- */
    while (1) {
        uint32_t delay_ms = ui_tick();
        if (delay_ms < LVGL_UPDATE_PERIOD_MS) delay_ms = LVGL_UPDATE_PERIOD_MS;

        // UI 触发 ECG 启动
        if (ui_is_ecg_active() && task_ecg == NULL) {
            xTaskCreatePinnedToCore(ecg_task, "ecg_sample", 16384, NULL,
                                    5, &task_ecg, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
