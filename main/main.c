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
static TaskHandle_t task_buzzer   = NULL;
static TaskHandle_t task_warn     = NULL;

/* ---- 状态 ---- */
static volatile bool sampling_active = false;
static volatile bool alarm_active    = false;
static float          current_hr     = 0.0f;

/* ---- 阈值 ---- */
static uint16_t alarm_high = HIGH_ALARM_DEFAULT;
static uint16_t alarm_low  = LOW_ALARM_DEFAULT;

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
 * 蜂鸣器警告任务
 * ============================================================ */
static void buzzer_task(void *pv)
{
    esp_task_wdt_add(NULL);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (alarm_active && sampling_active) {
            ESP_LOGW(TAG, "心率异常: %.1f BPM", (double)current_hr);
            ui_update_warning("心率异常", true);
            for (int i = 0; i < 3; i++) {
                hw_buzzer_on(2000);
                vTaskDelay(pdMS_TO_TICKS(100));
                hw_buzzer_off();
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        } else if (!alarm_active && sampling_active) {
            ui_update_warning("心率正常", false);
        }
        esp_task_wdt_reset();
    }
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

/* ============================================================
 * 警告标签闪烁任务
 * ============================================================ */
static void warn_blink_task(void *pv)
{
    esp_task_wdt_add(NULL);
    for (int i = 0; i < 3; i++) {
        ui_update_warning("", true);
        vTaskDelay(pdMS_TO_TICKS(300));
        // 恢复由 buzzer_task 负责
        vTaskDelay(pdMS_TO_TICKS(700));
        esp_task_wdt_reset();
    }
    esp_task_wdt_delete(NULL);
    task_warn = NULL;
    vTaskDelete(NULL);
}

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
                // 心率更新
                if (dsp_res.heart_rate > 0) {
                    current_hr = dsp_res.heart_rate;
                    if (dsp_res.qrs_count >= 5) {
                        ui_update_heart_rate(current_hr);
                    }

                    // 报警检测
                    alarm_active = (current_hr > alarm_high ||
                                    current_hr < alarm_low);

                    // AI 诊断
                    ecg_diag_result_t ai_res;
                    if (ecg_ai_process(&dsp_res, &ai_res) && ai_res.is_valid) {
                        ESP_LOGI(TAG, "AI: %s (置信度: %.2f)",
                                 ai_res.text, (double)ai_res.confidence);
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

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 停止并清理
    hw_adc_stop();
    hw_adc_deinit();
    ui_set_sampling_indicator(false);
    ui_update_heart_rate(0);
    current_hr = 0.0f;
    alarm_active = false;
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

    /* ---- 创建蜂鸣器任务 ---- */
    xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 4, &task_buzzer);

    /* ---- 主循环 ---- */
    while (1) {
        uint32_t delay_ms = ui_tick();
        if (delay_ms < LVGL_UPDATE_PERIOD_MS) delay_ms = LVGL_UPDATE_PERIOD_MS;

        // UI 触发 ECG 启动
        if (ui_is_ecg_active() && task_ecg == NULL) {
            xTaskCreatePinnedToCore(ecg_task, "ecg_sample", 8192, NULL,
                                    5, &task_ecg, 1);
        }

        // 报警闪烁
        if (alarm_active && task_warn == NULL && sampling_active) {
            xTaskCreatePinnedToCore(warn_blink_task, "warn_blink", 2048,
                                    NULL, 3, &task_warn, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
