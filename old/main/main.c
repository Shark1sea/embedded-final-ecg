#if CONFIG_FREERTOS_UNICORE == 0 && !defined(portYIELD_CORE)
#define portYIELD_CORE(coreid) do { esp_crosscore_int_send_yield(coreid); } while (0)
#endif
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_freertos_hooks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9488.h"
#include "esp_dsp.h"
#include "lv_conf.h"
#include "JiDianHei_18.c"
#include "panTompkins/panTompkins.h"
#define LV_FONT_DECLARE(JiDianHei_18) extern const lv_font_t JiDianHei_18; // 声明字体宏（用于LVGL字体声明）
/*
    以下是结构体类型定义
*/
// ECG 状态结构
typedef  struct{
    bool is_sampling; // ADC是否正在运行
    bool is_transmitting; // 是否正在传输数据
}ecg_state_t;
typedef struct {
    bool is_LON_LOW; // ADC状态负引脚状态
    bool is_LOP_LOW; // ADC状态正引脚状态
    bool is_control_HIGH; // ADC控制引脚状态
}adc_chip_state_t; // ADC芯片状态结构
/*
    以上是结构体的类型定义
*/

/*
    以下是全局变量定义
*/
/*
    程序变量
*/
static const char *TAG = "main";                // 日志标签
static const char *TAG_ADC = "adc";             // 日志标签
static const char *TAG_DISPLAY = "display";     // 日志标签
static const char *TAG_TOUCH = "touch";         // 日志标签
static const char *TAG_LVGL = "lvgl";           // 日志标签
static const char *TAG_ECG = "ecg";             // 日志标签
static const char *TAG_GPIO = "gpio";           // 日志标签
static const char *TAG_SPI = "spi";             // 日志标签
static const char *TAG_BUZZER = "buzzer";       // 日志标签

// ECG状态
volatile float heart_rate = 0.0f; // 心率（单位：BPM）
static const uint8_t max_high_alarm = 200; // 最大高心率报警阈值（单位：BPM）
static const uint8_t min_high_alarm = 120; // 最小高心率报警阈值（单位：BPM）
// static const uint8_t max_low_alarm = 70; // 最大低心率报警阈值（单位：BPM）
static const uint8_t min_low_alarm = 30; // 最小低心率报警阈值（单位：BPM）
static uint8_t high_alarm = 180; // 高心率报警阈值（单位：BPM）
#define HIGH_ALARM_DEFAULT 180 // 默认高心率报警阈值（单位：BPM）
static uint8_t low_alarm = 40; // 低心率报警阈值（单位：BPM）
#define LOW_ALARM_DEFAULT 40 // 默认低心率报警阈值（单位：BPM）
volatile bool is_alarm = false; // 是否触发报警
static uint8_t qrs_count = 0; // QRS波计数
volatile ecg_state_t ecg_state = {
    .is_sampling = false,
    .is_transmitting = false
};
// ADC芯片状态
volatile adc_chip_state_t adc_chip_state = {
    .is_LON_LOW = false, // ADC状态负引脚初始状态
    .is_LOP_LOW = false, // ADC状态正引脚初始状态
    .is_control_HIGH = false // ADC控制引脚初始状态
};
/*
    引脚定义
*/
#define ADC_GPIO_NUM            GPIO_NUM_34 // ADC引脚（GPIO34）
#define ADC_LON_GPIO_NUM        GPIO_NUM_35 // ADC状态负引脚（GPIO35） 低电平有效
#define ADC_LOP_GPIO_NUM        GPIO_NUM_32 // ADC状态正引脚（GPIO32） 低电平有效
#define ADC_CONTROL_GPIO_NUM    GPIO_NUM_25 // ADC控制引脚（GPIO25） 低电平有效

#define DISPLAY_SPI_CLK         GPIO_NUM_14 // 显示屏时钟引脚
#define DISPLAY_SPI_CS          GPIO_NUM_16 // 显示屏片选引脚
#define DISPLAY_SPI_MOSI        GPIO_NUM_15 // 显示屏spi输入
#define DISPLAY_SPI_MISO        GPIO_NUM_2  // 显示屏spi输出
#define DISPLAY_DC              GPIO_NUM_17 // 显示屏数据/命令引脚
#define DISPLAY_BACKLIGHT       GPIO_NUM_18 // 显示屏背光引脚
#define DISPLAY_RESET           GPIO_NUM_4  // 显示屏复位引脚 

#define TOUCH_SPI_CLK           GPIO_NUM_12 // 触摸屏时钟引脚
#define TOUCH_SPI_CS            GPIO_NUM_21 // 触摸屏片选引脚
#define TOUCH_SPI_MOSI          GPIO_NUM_23 // 触摸屏spi输入
#define TOUCH_SPI_MISO          GPIO_NUM_19 // 触摸屏spi输出

#define BUZZER_GPIO_NUM         GPIO_NUM_13 // 蜂鸣器引脚（GPIO13）
/* 
    蜂鸣器相关参数 
*/
#define BUZZER_LEDC_TIMER      LEDC_TIMER_2
#define BUZZER_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_CHANNEL    LEDC_CHANNEL_1
#define BUZZER_LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define BUZZER_LEDC_FREQ  2000 // 蜂鸣器PWM频率（Hz）
static TaskHandle_t buzzer_warning_Task_handle = NULL; // 蜂鸣器警告任务句柄
/*
    ADC相关参数和变量
*/
#define ADC_UNIT                    ADC_UNIT_1                  // 使用ADC1单元
#define _ADC_UNIT_STR(unit)         #unit                       // 将单元名转为字符串（辅助宏） 
#define ADC_UNIT_STR(unit)          _ADC_UNIT_STR(unit)         // 获取单元名字符串
#define ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1      // 单ADC模式（仅ADC1）
#define ADC_ATTEN                   ADC_ATTEN_DB_12             // 衰减（12dB）
#define ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH   // ADC最大位宽（芯片相关）

/* 数据输出格式 */
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2                
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1// ESP32/S2使用Type1格式
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)   // 从Type1数据中提取通道号
#define ADC_GET_DATA(p_data)        ((p_data)->type1.data)      // 从Type1数据中提取采样值
#endif

#define READ_LEN                    256                         // 每次读取的DMA数据长度（字节）
#define DEFAULT_VREF                3300                        // 默认参考电压（mV）
static uint32_t adc_sample_freq = 20 * 1000;                    // ADC采样频率（20kHz）
#define ADC_SAMPLE_FREQ             adc_sample_freq             

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_6};              // ESP32使用通道6(GPIO34)
#endif

static TaskHandle_t ecg_sample_task_handle = NULL;              // ECG采样任务句柄
adc_continuous_handle_t adc_handle = NULL;                      // ADC连续模式句柄
#define DOWNSAMPLE_FACTOR 36                                    // 12800/360 ≈ 36                     
static uint16_t downsample_count = 0;

#define FIR_LEN 64 // FIR滤波器长度（64个系数）
static fir_s16_t fir; // FIR滤波器结构体
static int16_t fir_coeffs[FIR_LEN] = {
    0, 1, 2, 6, 11, 19, 29, 43, 
    61, 83, 110, 142, 180, 224, 275, 332, 
    394, 463, 536, 613, 692, 773, 854, 934, 
    1010, 1081, 1146, 1202, 1249, 1286, 1310, 1323, 
    1323, 1310, 1286, 1249, 1202, 1146, 1081, 1010, 
    934, 854, 773, 692, 613, 536, 463, 394, 
    332, 275, 224, 180, 142, 110, 83, 61, 
    43, 29, 19, 11, 6, 2, 1, 0
};// FIR滤波器系数数组
static int16_t fir_delay[FIR_LEN] = {0}; // FIR滤波器延迟线数组
/*
    ili9488 LCD相关变量
*/
// 取消注释以下行以启用 LVGL 颜色数据的双缓冲
#define USE_DOUBLE_BUFFERING 1
// 显示屏分辨率和参数定义
static const int DISPLAY_HORIZONTAL_PIXELS = 480;
static const int DISPLAY_VERTICAL_PIXELS = 320;
static const int DISPLAY_COMMAND_BITS = 8;
static const int DISPLAY_PARAMETER_BITS = 8;
// 有些开发者需要降低此值以避免屏幕杂点，通常与 SPI 设备数量或连线长度有关。
// 如果出现杂点，尝试降低为 4 * 1000 * 1000，然后逐步增大直到杂点消失。
static const unsigned int DISPLAY_REFRESH_HZ = 40000000;
static const int DISPLAY_SPI_QUEUE_LEN = 10;
static const int DISPLAY_SPI_MAX_TRANSFER_SIZE = 32768;
static const lcd_rgb_element_order_t TFT_COLOR_MODE = COLOR_RGB_ELEMENT_ORDER_BGR;
// LVGL 缓冲区大小，默认 25 行
static const size_t LV_BUFFER_SIZE = DISPLAY_HORIZONTAL_PIXELS * 25;
static const int LVGL_UPDATE_PERIOD_MS = 5;
static esp_lcd_panel_io_handle_t lcd_io_handle = NULL; // SPI 总线句柄
static esp_lcd_panel_handle_t lcd_handle = NULL; // LCD 面板句柄
/*
    触摸芯片 XPT2046 相关变量
*/
#define XPT2046_CMD_X_READ  0x90
#define XPT2046_CMD_Y_READ  0xD0
static spi_device_handle_t touch_spi_handle = NULL;     // 触摸屏 SPI 设备句柄
static const int TOUCH_SPI_MAX_TRANSFER_SIZE = 4096;    // 触摸屏 SPI 最大传输大小
static lv_indev_drv_t indev_drv;// 初始化驱动结构体
/*
    背光 PWM 配置
*/
static const ledc_mode_t BACKLIGHT_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_channel_t BACKLIGHT_LEDC_CHANNEL = LEDC_CHANNEL_0;
static const ledc_timer_t BACKLIGHT_LEDC_TIMER = LEDC_TIMER_1;
static const ledc_timer_bit_t BACKLIGHT_LEDC_TIMER_RESOLUTION = LEDC_TIMER_10_BIT;
static const uint32_t BACKLIGHT_LEDC_FRQUENCY = 5000;
#define BACKLIGHT_DEFAULT_BRIGHTNESS 80 // 默认背光亮度（百分比，0-100）
static uint8_t backlight_brightness = 80; // 背光亮度（百分比，0-100）
static const uint8_t BACKLIGHT_BRIGHTNESS_MAX = 100; // 背光最大亮度（百分比）
static const uint8_t BACKLIGHT_BRIGHTNESS_MIN = 50; // 背光最小亮度（百分比）
/*
    LVGL 相关变量
*/
static lv_disp_draw_buf_t lv_disp_buf;      // LVGL 显示缓冲区
static lv_disp_drv_t lv_disp_drv;           // LVGL 显示驱动
static lv_disp_t *lv_display = NULL;        // LVGL 显示对象
static lv_color_t *lv_buf_1 = NULL;         // LVGL 缓冲区1
static lv_color_t *lv_buf_2 = NULL;         // LVGL 缓冲区2（如果启用双缓冲）
static TaskHandle_t warning_label_Task_handle = NULL; // 警告标签任务句柄
// LVGL 控件
static lv_obj_t *scr = NULL;                // LVGL 屏幕对象
static lv_style_t style_screen;             // LVGL 屏幕样式
static int16_t column_dsc[] = { 64,  DISPLAY_HORIZONTAL_PIXELS - 64, LV_GRID_TEMPLATE_LAST };// 列宽度
static int16_t row_dsc[] = { DISPLAY_VERTICAL_PIXELS* 0.5, DISPLAY_VERTICAL_PIXELS * 0.5, LV_GRID_TEMPLATE_LAST };// 行高度
static const uint8_t menu_btn_height = 120;
static lv_obj_t *menu_cont = NULL;          // LVGL 菜单容器对象
static int16_t menu_col_dsc[] = { 60, 4, LV_GRID_TEMPLATE_LAST };
static int16_t menu_row_dsc[] = { 320, LV_GRID_TEMPLATE_LAST };
static lv_obj_t *menu_btn_cont = NULL;      // LVGL 菜单按钮容器对象
static lv_obj_t *menu_btn_1 = NULL;         // LVGL 菜单按钮1对象
static lv_obj_t *menu_btn_1_label = NULL;   // LVGL 菜单按钮1标签对象
static lv_obj_t *menu_btn_2 = NULL;         // LVGL 菜单按钮2对象
static lv_obj_t *menu_btn_2_label = NULL;   // LVGL 菜单按钮2标签对象
static lv_style_t style_menu_btn;           // LVGL 菜单按钮样式
static lv_style_t usual_label_style;        // LVGL 普通标签样式
static lv_obj_t * indi_cont = NULL;         // LVGL 指示器容器对象
static lv_obj_t *indicator = NULL;          // LVGL 指示器对象
static lv_obj_t *cont = NULL;               // LVGL 容器对象
static lv_obj_t *page_1 = NULL;             // LVGL 页面1对象
static int16_t column_dsc_1[] = { 136, 136, 144, LV_GRID_TEMPLATE_LAST };// 列宽度
static int16_t row_dsc_1[] = { 60, 50, 210, LV_GRID_TEMPLATE_LAST };// 行高度
static lv_obj_t *page_2 = NULL;             // LVGL 页面2对象
static lv_obj_t *ecg_btn;                   // LVGL ECG按钮对象
static lv_obj_t *ecg_btn_label = NULL;      // LVGL ECG按钮标签对象
static lv_style_t style_ecg_start;          // LVGL ECG开始按钮样式
static lv_style_t style_ecg_stop;           // LVGL ECG停止按钮样式
static lv_obj_t *transmit_btn;              // LVGL 传输按钮对象
static lv_obj_t *tarnsmit_btn_label = NULL; // LVGL 传输按钮标签对象
static lv_style_t style_transmit_start;     // LVGL 传输开始按钮样式
static lv_style_t style_tarnsmit_stop;      // LVGL 传输停止按钮样式
static lv_obj_t *freq_label = NULL;                // LVGL LED对象
static lv_obj_t *waring_label = NULL;       // LVGL 警告标签对象
static lv_style_t style_waring_label;       // LVGL 警告标签样式
static lv_style_t style_waring_label_no_warning; // LVGL 无警告标签样式
static lv_obj_t *hr_label = NULL;           // LVGL 心率标签对象
static lv_style_t style_hr_label;           // LVGL 心率标签样式
static lv_obj_t *ecg_cont = NULL;           // LVGL ECG容器对象
static lv_point_t ecg_points[396];          // ECG数据点数组（396个点）
static lv_obj_t *ecg_line = NULL;           // LVGL ECG线条对象
static lv_style_t style_ecg_line;           // LVGL ECG线条样式
static uint16_t ecg_point_index = 0;        // 静态变量，记录当前点索引
static lv_obj_t *ecg_indicator = NULL;      // LVGL ECG指示器对象
static lv_style_t style_ecg_indicator;      // LVGL ECG指示器样式
static int16_t column_dsc_2[] = { 416, LV_GRID_TEMPLATE_LAST }; // 页面2列宽度
static int16_t row_dsc_2[] = { 60, 260, LV_GRID_TEMPLATE_LAST}; // 页面2行高度
static lv_obj_t *setting_head_cont = NULL;  // LVGL 设置头部容器对象
static lv_obj_t *setting_head_label = NULL; // LVGL 设置头部标签对象
static lv_obj_t *reset_btn = NULL;          // LVGL 重置按钮对象
static lv_obj_t *reset_btn_label = NULL;   // LVGL 重置按钮标签对象
static lv_style_t style_reset_btn;         // LVGL 重置按钮样式
static lv_obj_t *setting_cont = NULL;       // LVGL 设置容器对象
static lv_obj_t *setting_list = NULL;       // LVGL 设置列表对象
static lv_obj_t *brightness_item = NULL;    // LVGL 背光亮度列表项对象
static lv_obj_t *brightness_label = NULL;   // LVGL 背光亮度标签对象
static lv_obj_t *brightness_bar = NULL;     // LVGL 背光亮度滑块对象
static lv_obj_t *brightness_value_label = NULL; // LVGL 背光亮度值标签对象
static lv_obj_t *high_alarm_item = NULL;         // LVGL 报警设置列表项对象
static lv_obj_t *high_alarm_label = NULL;   // LVGL 高心率报警标签对象
static lv_obj_t *high_alarm_value_label = NULL; // LVGL 高心率报警值标签对象
static lv_obj_t *high_alarm_bar = NULL;     // LVGL 高心率报警滑块对象
static lv_obj_t *low_alarm_item = NULL;          // LVGL 低心率报警设置列表项对象
static lv_obj_t *low_alarm_label = NULL;    // LVGL 低心率报警标签对象
static lv_obj_t *low_alarm_value_label = NULL; // LVGL 低心率报警值标签对象
static lv_obj_t *low_alarm_bar = NULL;      // LVGL 低心率报警滑块对象
/*
    以上是全局变量定义
*/

/*
    函数声明
*/
// 初始化GPIO引脚
static void gpio_init(); 
// 初始化SPI总线
static void spi_init(); 
// 读取ADC芯片状态
static void read_adc_chip_state(); 
// ADC连续模式转换完成回调函数
static bool IRAM_ATTR adc_conviuous_done_cb(adc_continuous_handle_t handle, 
    const adc_continuous_evt_data_t *edata, void *user_data); 
// 初始化ADC连续模式驱动
static void continuous_adc_init(adc_channel_t *channel, 
    uint8_t channel_num, adc_continuous_handle_t *out_handle); 
// 初始化蜂鸣器
static void buzzer_init();
// 蜂鸣器发声函数
static void buzzer_beep_on(uint32_t frequency_hz);
static void buzzer_beep_off(); // 蜂鸣器停止发声
// 初始化显示屏
static void display_init(); 
// 初始化触摸芯片
static void touch_init(); 
// 通知 LVGL 刷新完成的回调
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, 
    esp_lcd_panel_io_event_data_t *edata, void *user_ctx); // SPI传输完成后通知LVGL刷新完成
// 初始化背光PWM
static void display_brightness_init(void); // 初始化背光PWM
// 设置背光亮度
static void display_brightness_set(int brightness_percentage); // 设置背光亮度（百分比）
// 触摸芯片读取
static bool read_touch_chip(uint16_t *x, uint16_t *y);
// 初始化LVGL触摸输入设备
static void lvgl_touch_init();
// 触摸芯片读取函数
static void touch_lvgl_read(lv_indev_drv_t *drv, lv_indev_data_t *data);
// lvgl刷新回调
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
// LVGL 定时器回调，周期性增加 LVGL tick
static void IRAM_ATTR lvgl_tick_cb(void *param);
// 初始化 LVGL 图形库
static void lvgl_init();
// 初始化样式
static void lvgl_style_init();
// 创建UI
static void create_UI();
// 添加内容到页面1
static void add_content_to_page_1(lv_obj_t *cont);
// 添加内容到页面2
static void add_content_to_page_2(lv_obj_t *cont);
// 重设警告标签
static void set_warning_label(const char *text, const bool is_warning);
// 菜单按钮事件回调函数
static void menu_btn_event_cb(lv_event_t *e);
// ECG按钮事件回调函数
static void ecg_btn_event_cb(lv_event_t *e);
// 传输按钮事件回调函数
static void transmit_btn_event_cb(lv_event_t *e);
// 重置按钮事件回调函数
static void reset_btn_event_cb(lv_event_t *e);
// 设置背光亮度事件回调函数
static void brightness_bar_event_cb(lv_event_t *e);
// 设置高心率报警事件回调函数
static void high_alarm_bar_event_cb(lv_event_t *e);
// 设置低心率报警事件回调函数
static void low_alarm_bar_event_cb(lv_event_t *e);
// 将uint16_t转换为int16_t
static int16_t uint16_to_int16(uint16_t x);
// 将int16_t转换为uint16_t
static uint16_t int16_to_uint16(int16_t y);
// 蜂鸣器警告任务函数
static void buzzer_warning_Task();
// 警告标签任务函数
static void warning_label_Task();
// ECG采样任务
static void ecg_sample_Task(); // ECG采样任务函数
/*
    函数声明
*/

/**
 * @brief 主应用程序入口函数
 * @note 初始化ADC、GPIO、LVGL等，并启动ADC连续采样
 * @return void
 *
**/
void app_main(void)
{
    /* 初始化 */
    gpio_init();                    // 初始化GPIO引脚
    spi_init();                     // 初始化SPI总线
    buzzer_init();                  // 初始化蜂鸣器
    display_brightness_init();      // 初始化背光
    display_brightness_set(0);      // 先关闭背光 
    display_init();                 // 初始化显示屏
    touch_init();                   // 初始化触摸芯片
    lvgl_init();                    // 初始化LVGL图形库 
    lvgl_touch_init();              // 初始化触摸LVGL绑定
    lvgl_style_init();              // 初始化LVGL样式
    create_UI();                    // 创建UI界面
    display_brightness_set(80);     // 设置背光为 80%
    lv_event_send(menu_btn_1, LV_EVENT_CLICKED, NULL); // 发送点击事件以更新位置
    ESP_ERROR_CHECK(gpio_set_level(ADC_CONTROL_GPIO_NUM, 0)); // 启用低功耗
    ESP_LOGI(TAG, "系统初始化完成");
    xTaskCreate(buzzer_warning_Task, "蜂鸣器警告任务", 2048, NULL, 4, &buzzer_warning_Task_handle); // 创建蜂鸣器警告任务
    //is_alarm = true;
    while (1)  
    {
        vTaskDelay(pdMS_TO_TICKS(10)); // 每10ms执行一次LVGL刷新
        lv_timer_handler();
        if(ecg_state.is_sampling && ecg_sample_task_handle == NULL) // 如果正在采样且任务未创建
        {
            // 创建ECG采样任务
            xTaskCreatePinnedToCore(ecg_sample_Task, "ECG采样任务", 8192, NULL, 5, &ecg_sample_task_handle, 1);
        }
    }
    ESP_LOGI(TAG, "应用程序已退出");
}


/*
    以下是函数定义
*/
/**
 * @brief 初始化GPIO引脚
 * @note 配置ADC状态引脚、控制引脚和蜂鸣器引脚
 * @return void
**/
static void gpio_init()
{
    gpio_config_t adc_lo_gpio_config = {
        .pin_bit_mask = (1ULL << ADC_LON_GPIO_NUM) | (1ULL << ADC_LOP_GPIO_NUM),
        .mode = GPIO_MODE_INPUT, // 设置为输入模式
        .pull_up_en = GPIO_PULLUP_ENABLE, // 启用上拉电阻
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉电阻
        .intr_type = GPIO_INTR_DISABLE // 禁用中断
    };
    gpio_config_t adc_control_gpio_config = {
        .pin_bit_mask = (1ULL << ADC_CONTROL_GPIO_NUM),
        .mode = GPIO_MODE_OUTPUT, // 设置为输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE, // 禁用上拉电阻
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉电阻
        .intr_type = GPIO_INTR_DISABLE // 禁用中断
    };
    gpio_config_t buzzer_gpio_config = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO_NUM), // 设置蜂鸣器引脚
        .mode = GPIO_MODE_OUTPUT, // 设置为输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE, // 禁用上拉电阻
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉电阻
        .intr_type = GPIO_INTR_DISABLE // 禁用中断
    };
    ESP_ERROR_CHECK(gpio_config(&adc_lo_gpio_config)); // 配置ADC状态引脚
    ESP_ERROR_CHECK(gpio_config(&adc_control_gpio_config)); // 配置ADC控制引脚
    ESP_ERROR_CHECK(gpio_config(&buzzer_gpio_config)); // 配置蜂鸣器引脚
    ESP_ERROR_CHECK(gpio_set_level(ADC_CONTROL_GPIO_NUM, 0)); // 设置控制引脚高电平（关闭ADC）
    ESP_LOGI(TAG_GPIO, "GPIO初始化完成: ADC状态引脚: LON=%d, LOP=%d, 控制引脚: %d",
             ADC_LON_GPIO_NUM, ADC_LOP_GPIO_NUM, ADC_CONTROL_GPIO_NUM);
    // 设置ADC控制引脚初始状态为高电平（关闭ADC）
}
/**
 * @brief 初始化SPI总线
 * @note 配置显示屏和触摸屏的SPI总线
 * @return void
 **/
static void spi_init()
{
    // 初始化 SPI 总线配置
    spi_bus_config_t display_bus =
    {
        .mosi_io_num = DISPLAY_SPI_MOSI,
        .miso_io_num = DISPLAY_SPI_MISO,
        .sclk_io_num = DISPLAY_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,
        .max_transfer_sz = DISPLAY_SPI_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO |
                 SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &display_bus, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG_SPI, "初始化 SPI2 总线 (MOSI:%d, MISO:%d, CLK:%d), 线上设备: ili9488",
             DISPLAY_SPI_MOSI, DISPLAY_SPI_MISO, DISPLAY_SPI_CLK);
    // 初始化触摸屏 SPI 总线配置
    spi_bus_config_t touch_bus =
    {
        .mosi_io_num = TOUCH_SPI_MOSI,
        .miso_io_num = TOUCH_SPI_MISO,
        .sclk_io_num = TOUCH_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,
        .max_transfer_sz = TOUCH_SPI_MAX_TRANSFER_SIZE, // 触摸屏最大传输大小
        .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO |
                 SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &touch_bus, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG_SPI, "初始化 SPI3 总线 (MOSI:%d, MISO:%d, CLK:%d), 线上设备: 触控芯片XPT2046",
             TOUCH_SPI_MOSI, TOUCH_SPI_MISO, TOUCH_SPI_CLK);
}
/**
 * @brief 初始化蜂鸣器
 * @note 配置蜂鸣器的定时器和通道
 * @return void
 */
static void buzzer_init()
{
    // 配置定时器
    ledc_timer_config_t buzzer_timer = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .timer_num        = BUZZER_LEDC_TIMER,
        .duty_resolution  = BUZZER_LEDC_DUTY_RES,
        .freq_hz          = BUZZER_LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&buzzer_timer));

    // 配置通道
    ledc_channel_config_t buzzer_channel = {
        .gpio_num       = BUZZER_GPIO_NUM,
        .speed_mode     = BUZZER_LEDC_MODE,
        .channel        = BUZZER_LEDC_CHANNEL,
        .timer_sel      = BUZZER_LEDC_TIMER,
        .duty           = 0, // 初始占空比为0（静音）
        .hpoint         = 0,
        .flags.output_invert = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&buzzer_channel));
    ESP_LOGI(TAG_BUZZER, "蜂鸣器初始化完成: GPIO=%d, 频率=%d Hz", BUZZER_GPIO_NUM, BUZZER_LEDC_FREQ);

}
/**
 * @brief 蜂鸣器发声函数
 * @param frequency_hz 频率（Hz）
 */
static void buzzer_beep_on(uint32_t frequency_hz)
{
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, frequency_hz);
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, (1 << BUZZER_LEDC_DUTY_RES) / 2);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}
static void buzzer_beep_off()
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}
/*
    以下是ADC相关函数定义
*/
/**
 * @brief 读取ADC芯片状态
 * @note 读取ADC芯片状态引脚的电平状态，并打印状态信息
 */
static void read_adc_chip_state()
{
    // 读取ADC芯片状态引脚
    adc_chip_state.is_LON_LOW = !gpio_get_level(ADC_LON_GPIO_NUM); // 读取负引脚状态
    adc_chip_state.is_LOP_LOW = !gpio_get_level(ADC_LOP_GPIO_NUM); // 读取正引脚状态
    adc_chip_state.is_control_HIGH = gpio_get_level(ADC_CONTROL_GPIO_NUM); // 读取控制引脚状态

    // 打印ADC芯片状态
    ESP_LOGI(TAG_ADC, "ADC 芯片状态: LON=%d, LOP=%d, Control=%d",
             adc_chip_state.is_LON_LOW, adc_chip_state.is_LOP_LOW, adc_chip_state.is_control_HIGH);
}
/**
  * @brief ADC连续模式转换完成回调函数（中断上下文）
  * @param handle ADC句柄
  * @param edata 事件数据（未使用）
  * @param user_data 用户数据（未使用）
  * @return 是否需要任务切换（由FreeRTOS决定）
  */
static bool IRAM_ATTR adc_conviuous_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // 通知ADC连续模式驱动已完成转换
    vTaskNotifyGiveFromISR(ecg_sample_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

/**
  * @brief 初始化ADC连续模式驱动
  * @param channel 通道数组
  * @param channel_num 通道数量
  * @param out_handle 输出ADC句柄
  */
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    /* 配置ADC句柄参数 */
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 8192,     // DMA缓冲区大小（字节）
        .conv_frame_size = READ_LEN,    // 每帧数据长度（字节）
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));   // 创建句柄
    /* 配置ADC采样参数 */
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_FREQ,  // 采样率
        .conv_mode = ADC_CONV_MODE,         // 转换模式（单ADC单元）
        .format = ADC_OUTPUT_TYPE,          // 数据输出格式（Type1/Type2）
    };
    /* 配置多通道采样模式 */
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;          // 通道数量
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;               // 衰减（0dB）
        adc_pattern[i].channel = channel[i] & 0x7;              // 通道号（取低3位）
        adc_pattern[i].unit = ADC_UNIT;                 // ADC单元（ADC1）
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;       // 位宽（12-bit）

        ESP_LOGI(TAG_ADC, "ADC单元: %s, 通道号: %d, 衰减: 12 dB, 位宽: %d bit",
                 ADC_UNIT_STR(ADC_UNIT), adc_pattern[i].channel, ADC_BIT_WIDTH);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));   // 应用配置
    *out_handle = handle;       // 返回初始化完成的句柄
}
/*
    以下是显示屏相关函数定义
*/
/**
 * @brief 通知 LVGL 刷新完成的回调函数
 * @param panel_io LCD面板IO句柄
 * @param edata 事件数据（未使用）
 * @param user_ctx 用户上下文（LVGL显示驱动）
 * @return 是否需要继续处理事件（通常返回false）
**/
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
/**
 * @brief 初始化背光 PWM
 * @param None
 * @return void
**/
static void display_brightness_init(void)
{
    const ledc_channel_config_t LCD_backlight_channel =
    {
        .gpio_num = DISPLAY_BACKLIGHT,
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .channel = BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = 
        {
            .output_invert = 0
        }
    };
    const ledc_timer_config_t LCD_backlight_timer =
    {
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .duty_resolution = BACKLIGHT_LEDC_TIMER_RESOLUTION,
        .timer_num = BACKLIGHT_LEDC_TIMER,
        .freq_hz = BACKLIGHT_LEDC_FRQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_LOGI(TAG_DISPLAY, "初始化背光 PWM, 管脚: %d", DISPLAY_BACKLIGHT);

    ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));
}
/**
 * @brief 设置背光亮度
 * @param brightness_percentage 背光亮度百分比（0-100）
 * @return void
 */
static void display_brightness_set(int brightness_percentage)
{
    if (brightness_percentage > 100)
    {
        brightness_percentage = 100;
    }    
    else if (brightness_percentage < 0)
    {
        brightness_percentage = 0;
    }
    ESP_LOGI(TAG_DISPLAY, "设置背光亮度为 %d%%", brightness_percentage);
    // LEDC 分辨率为 10 位，100% = 1023
    uint32_t duty_cycle = (1023 * brightness_percentage) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL));
}
/** 
 * @brief 初始化显示屏
 * @param None
 * @return void
**/
static void display_init()
{
    // 初始化 SPI 总线
    const esp_lcd_panel_io_spi_config_t io_config = 
    {
        .cs_gpio_num = DISPLAY_SPI_CS,
        .dc_gpio_num = DISPLAY_DC,
        .spi_mode = 0,
        .pclk_hz = DISPLAY_REFRESH_HZ,
        .trans_queue_depth = DISPLAY_SPI_QUEUE_LEN,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &lv_disp_drv,
        .lcd_cmd_bits = DISPLAY_COMMAND_BITS,
        .lcd_param_bits = DISPLAY_PARAMETER_BITS,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .flags =
        {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0
        }
    };
    // 初始化 SPI IO 配置
    const esp_lcd_panel_dev_config_t lcd_config = 
    {
        .reset_gpio_num = DISPLAY_RESET,
        .color_space = TFT_COLOR_MODE,
        .bits_per_pixel = 18,
        .flags =
        {
            .reset_active_high = 0
        },
        .vendor_config = NULL
    };
    // 初始化显示屏 SPI IO
    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &lcd_io_handle)); 
    ESP_ERROR_CHECK(
        esp_lcd_new_panel_ili9488(lcd_io_handle, &lcd_config, LV_BUFFER_SIZE, &lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_handle, true));       // 交换X和Y轴
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_handle, true, false)); // 该语句将显示内容镜像
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handle, true));
    ESP_LOGI(TAG_DISPLAY, "初始化显示屏: ILI9488, 分辨率: %dx%d", DISPLAY_HORIZONTAL_PIXELS, DISPLAY_VERTICAL_PIXELS);
}
/*
    以下是触控芯片相关函数
*/
/**
 * @brief 初始化触摸芯片 SPI IO
 * @param None
**/
static void touch_init()
{
    // 初始化触摸芯片 SPI IO
    const spi_device_interface_config_t touch_io_config = 
    {
        .clock_speed_hz = 2 * 1000 * 1000, // 2MHz
        .mode = 0, // SPI 模式 0   
        .spics_io_num = TOUCH_SPI_CS, // 片选引脚
        .queue_size = 7, // 
        .pre_cb = NULL,
        .post_cb = NULL
    };
    // 添加触摸设备
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &touch_io_config, &touch_spi_handle));
}
/**
 * @brief 读取触摸芯片
 * @param x 
 */
static bool read_touch_chip(uint16_t *x, uint16_t *y) {
    uint8_t tx_buf[3] = {0};
    uint8_t rx_buf[3] = {0};
    
    spi_transaction_t trans = {
        .length = 3 * 8,  // 3字节
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    // 读取X坐标
    tx_buf[0] = XPT2046_CMD_X_READ;
    spi_device_transmit(touch_spi_handle, &trans);
    *x = ((rx_buf[1] << 8) | rx_buf[2]) >> 3;
    // 读取Y坐标
    tx_buf[0] = XPT2046_CMD_Y_READ;
    spi_device_transmit(touch_spi_handle, &trans);
    *y = ((rx_buf[1] << 8) | rx_buf[2]) >> 3;
    return (*x < 4095 && *y < 4095);  // 有效数据检查
}
//
/**
 * @brief lvgl读取触摸信息
 * @param drv 
 * @param data 数据
 */
static void touch_lvgl_read(lv_indev_drv_t *drv, lv_indev_data_t *data) 
{
    static lv_point_t last_point = {0};
    // 读取触摸状态和坐标
    uint16_t x, y;
    bool touched = read_touch_chip(&x, &y);
    // 坐标转换
    if(touched) {
        last_point.x = x * 480 / 4096;
        last_point.y = 320 - y * 320 / 4096;
        // 边界检查
        last_point.x = LV_CLAMP(0, last_point.x, 479);
        last_point.y = LV_CLAMP(0, last_point.y, 319);
    }
    // 填充数据
    data->point.x = last_point.x;
    data->point.y = last_point.y;
    data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}
/*
    LVGL相关函数
*/
/**
 * @brief LVGL 刷新回调函数
 * @param drv LVGL显示驱动
 * @param area 刷新区域
 * @param color_map 刷新颜色数据
 * @return void
 */
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}
/**
 * @brief LVGL 定时器回调函数
 * @param param 用户参数（未使用）
 * @return void
 */
static void IRAM_ATTR lvgl_tick_cb(void *param)
{
    lv_tick_inc(LVGL_UPDATE_PERIOD_MS);
}
// 初始化 LVGL 图形库
/**
 * @brief 初始化 LVGL 图形库
 * @param None
 * @return void
 */
static void lvgl_init()
{
    ESP_LOGI(TAG_LVGL, "初始化 LVGL");
    lv_init();
    ESP_LOGI(TAG_LVGL, "为 LVGL 缓冲区分配 %zu 字节", LV_BUFFER_SIZE * sizeof(lv_color_t));
    lv_buf_1 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#if USE_DOUBLE_BUFFERING
    ESP_LOGI(TAG, "为第二个 LVGL 缓冲区分配 %zu 字节", LV_BUFFER_SIZE * sizeof(lv_color_t));
    lv_buf_2 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#endif
    ESP_LOGI(TAG_LVGL, "创建 LVGL 显示缓冲区");
    lv_disp_draw_buf_init(&lv_disp_buf, lv_buf_1, lv_buf_2, LV_BUFFER_SIZE);

    ESP_LOGI(TAG_LVGL, "初始化 %dx%d 显示屏", DISPLAY_HORIZONTAL_PIXELS, DISPLAY_VERTICAL_PIXELS);
    lv_disp_drv_init(&lv_disp_drv);
    lv_disp_drv.hor_res = DISPLAY_HORIZONTAL_PIXELS;
    lv_disp_drv.ver_res = DISPLAY_VERTICAL_PIXELS;
    lv_disp_drv.flush_cb = lvgl_flush_cb;
    lv_disp_drv.draw_buf = &lv_disp_buf;
    lv_disp_drv.user_data = lcd_handle;
    lv_display = lv_disp_drv_register(&lv_disp_drv);

    ESP_LOGI(TAG_LVGL, "创建 LVGL 定时器");
    const esp_timer_create_args_t lvgl_tick_timer_args =
    {
        .callback = &lvgl_tick_cb,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL; // 定时器句柄
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer)); // 创建定时器
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_UPDATE_PERIOD_MS * 1000)); // 启动定时器
}
/**
 * @brief 初始化 LVGL 触摸输入设备
 * @param None
 * @return void
 */
static void lvgl_touch_init() {
    lv_indev_drv_init(&indev_drv);
    // 配置输入类型
    indev_drv.type = LV_INDEV_TYPE_POINTER; // 指针设备(触摸/鼠标)
    indev_drv.read_cb = touch_lvgl_read; // 设置读取回调
    lv_indev_t * touch_indev = lv_indev_drv_register(&indev_drv);// 注册输入设备
}
/**
 * @brief 创建 LVGL UI 界面
 * @param None
 * @return void
 * @note 该函数创建了主屏幕、菜单容器、菜单按钮等UI元素，并设置样式和布局
 */
static void create_UI()
{
    scr = lv_disp_get_scr_act(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);// 清除滚动标志
    lv_obj_add_style(scr, &style_screen, 0); // 设置屏幕样式
    // 设置活动屏幕 grid 布局
    lv_obj_set_grid_dsc_array(scr, column_dsc, row_dsc);
    // 在全局 grid 布局中去除边距
    lv_obj_set_style_pad_column(scr, 0, 0);
    lv_obj_set_style_pad_row(scr, 0, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    // 设置菜单容器
    menu_cont = lv_obj_create(scr);
    lv_obj_set_size(menu_cont, 64, DISPLAY_VERTICAL_PIXELS);
    lv_obj_set_style_bg_color(menu_cont, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(menu_cont, 0, 0);
    lv_obj_set_style_pad_all(menu_cont, 0, 0);
    lv_obj_set_style_radius(menu_cont, 0, 0); // 去除圆角
    lv_obj_clear_flag(menu_cont, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_grid_cell(menu_cont, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_CENTER, 0, 2);
    //设置菜单容器 grid 布局
    lv_obj_set_grid_dsc_array(menu_cont, menu_col_dsc, menu_row_dsc);
    // 在 menu_cont grid 布局中去除边距
    lv_obj_set_style_pad_column(menu_cont, 0, 0);
    lv_obj_set_style_pad_row(menu_cont, 0, 0);
    lv_obj_set_style_pad_all(menu_cont, 0, 0);
    // 设置菜单按钮容器
    menu_btn_cont = lv_obj_create(menu_cont);
    lv_obj_set_size(menu_btn_cont, 60, DISPLAY_VERTICAL_PIXELS);
    lv_obj_set_style_bg_color(menu_btn_cont, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(menu_btn_cont, 0, 0);
    lv_obj_set_style_pad_all(menu_btn_cont, 0, 0);
    lv_obj_set_style_radius(menu_btn_cont, 0, 0); // 去除圆角
    lv_obj_clear_flag(menu_btn_cont, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_flex_flow(menu_btn_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(menu_btn_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
    lv_obj_set_grid_cell(menu_btn_cont, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_CENTER, 0, 1);
    // 设置菜单按钮 1
    menu_btn_1 = lv_btn_create(menu_btn_cont);
    lv_obj_set_size(menu_btn_1, 60, menu_btn_height); // 设置按钮大小
    lv_obj_add_style(menu_btn_1, &style_menu_btn, 0); // 添加样式
    lv_obj_add_event_cb(menu_btn_1, menu_btn_event_cb, LV_EVENT_CLICKED, NULL);
    menu_btn_1_label = lv_label_create(menu_btn_1);
    lv_obj_set_align(menu_btn_1_label, LV_ALIGN_CENTER); // 设置标签对齐方式
    lv_label_set_text(menu_btn_1_label, "心\n电\n采\n集"); // 设置按钮文本
    lv_obj_add_style(menu_btn_1_label, &usual_label_style, 0); // 添加标签样式
    // 设置菜单按钮 2
    menu_btn_2 = lv_btn_create(menu_btn_cont);
    lv_obj_set_size(menu_btn_2, 60, menu_btn_height); // 设置按钮大小
    lv_obj_add_style(menu_btn_2, &style_menu_btn, 0); // 添加样式
    lv_obj_add_event_cb(menu_btn_2, menu_btn_event_cb, LV_EVENT_CLICKED, NULL);
    menu_btn_2_label = lv_label_create(menu_btn_2);
    lv_obj_set_align(menu_btn_2_label, LV_ALIGN_CENTER); // 设置标签对齐方式
    lv_label_set_text(menu_btn_2_label, "系\n统\n设\n置"); // 设置按钮文本
    lv_obj_add_style(menu_btn_2_label, &usual_label_style, 0); // 添加标签样式
    // 设置按钮指示线容器
    indi_cont = lv_obj_create(menu_cont);
    lv_obj_set_size(indi_cont, 4, DISPLAY_VERTICAL_PIXELS);
    lv_obj_set_style_bg_color(indi_cont, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(indi_cont, 0, 0);
    lv_obj_set_style_pad_all(indi_cont, 0, 0);
    lv_obj_set_style_radius(indi_cont, 0, 0); // 去除圆角
    lv_obj_clear_flag(indi_cont, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_grid_cell(indi_cont, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 0, 1);
    // 设置按钮指示线
    indicator = lv_obj_create(indi_cont);
    lv_obj_set_size(indicator, 4, 80); // 4像素宽，高度 80
    lv_obj_set_style_bg_color(indicator, lv_color_make(0x00, 0x80, 0xFF), 0); // 蓝色
    lv_obj_set_style_border_width(indicator, 0, 0);
    lv_obj_set_style_radius(indicator, 2, 0);
    // 设置主内容器
    cont = lv_obj_create(scr);
    lv_obj_set_size(cont, DISPLAY_HORIZONTAL_PIXELS * 0.8, DISPLAY_VERTICAL_PIXELS);
    lv_obj_set_style_bg_color(cont, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    // 主内容区同理去除边框和内边距
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_set_style_radius(cont, 0, 0); // 去除圆角
    lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 0, 2);
    // 
    page_1 = lv_obj_create(cont);
    lv_obj_set_size(page_1, lv_pct(100), lv_pct(100));
    lv_obj_set_style_pad_all(page_1, 0, 0);
    lv_obj_set_style_radius(page_1, 0, 0); // 去除圆角
    lv_obj_clear_flag(page_1, LV_OBJ_FLAG_SCROLLABLE);
    add_content_to_page_1(page_1);
    //
    page_2 = lv_obj_create(cont);
    lv_obj_set_size(page_2, lv_pct(100), lv_pct(100));
    lv_obj_set_style_pad_all(page_2, 0, 0);
    lv_obj_set_style_radius(page_2, 0, 0); // 去除圆角
    lv_obj_clear_flag(page_2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(page_2, LV_OBJ_FLAG_HIDDEN); // 初始隐藏
    add_content_to_page_2(page_2);
}
/**
 * @brief 添加页面1的内容
 * @param cont 页面1的容器对象
 * @note 该函数清除之前的内容，设置网格布局，添加心电按钮、传输按钮、LED指示灯、心率标签、警告标签和心电图容器等UI元素
 * @return void
 */
static void add_content_to_page_1(lv_obj_t *cont)
{
    // 清除之前的内容
    lv_obj_clean(cont);
    // 清除滚动标志
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE); 
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    // 设置子页面的网格布局
    lv_obj_set_grid_dsc_array(cont, column_dsc_1, row_dsc_1);
    // 在 cont grid 布局中去除边距  
    lv_obj_set_style_pad_column(cont, 0, 0);
    lv_obj_set_style_pad_row(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    // 设置心电按钮
    ecg_btn = lv_btn_create(cont);
    lv_obj_set_size(ecg_btn, 110, 50); // 设置按钮大小
    lv_obj_add_style(ecg_btn, &style_ecg_start, 0); // 添加样式
    lv_obj_add_event_cb(ecg_btn, ecg_btn_event_cb, LV_EVENT_CLICKED, NULL);
    ecg_btn_label = lv_label_create(ecg_btn);
    lv_label_set_text(ecg_btn_label, "开始采集"); // 设置按钮文本
    lv_obj_center(ecg_btn_label); // 标签居中
    lv_obj_add_style(ecg_btn_label, &usual_label_style, 0); // 添加标签样式
    lv_obj_set_grid_cell(ecg_btn, LV_GRID_ALIGN_CENTER, 0, 1, LV_GRID_ALIGN_CENTER, 0, 1);
    // 
    transmit_btn = lv_btn_create(cont);
    lv_obj_set_size(transmit_btn, 110, 50); // 设置按钮大小
    lv_obj_add_style(transmit_btn, &style_transmit_start, 0); // 添加样式
    lv_obj_add_event_cb(transmit_btn, transmit_btn_event_cb, LV_EVENT_CLICKED, NULL);
    tarnsmit_btn_label = lv_label_create(transmit_btn);
    lv_label_set_text(tarnsmit_btn_label, "串口传输"); // 设置按钮文本
    lv_obj_center(tarnsmit_btn_label); // 标签居中
    lv_obj_add_style(tarnsmit_btn_label, &usual_label_style, 0); // 添加标签样式
    lv_obj_set_grid_cell(transmit_btn, LV_GRID_ALIGN_CENTER, 1, 1, LV_GRID_ALIGN_CENTER, 0, 1);
    // 
    freq_label = lv_label_create(cont);
    lv_label_set_text(freq_label, "采样率: 360 Hz"); // 设置初始文本
    lv_obj_center(freq_label); // 标签居中
    lv_obj_add_style(freq_label, &usual_label_style, 0); // 添加样式
    lv_obj_set_grid_cell(freq_label, LV_GRID_ALIGN_CENTER, 2, 1, LV_GRID_ALIGN_CENTER, 0, 1);
    // 设置心率标签
    hr_label = lv_label_create(cont);
    lv_label_set_text(hr_label, "心率: -- bpm"); // 设置初始文本
    lv_obj_add_style(hr_label, &style_hr_label, 0); // 添加样式
    lv_obj_set_grid_cell(hr_label, LV_GRID_ALIGN_CENTER, 0, 1, LV_GRID_ALIGN_CENTER, 1, 1);
    // 设置警告标签
    waring_label = lv_label_create(cont);
    lv_label_set_text(waring_label, "请先连接导联"); // 设置初始文本
    lv_obj_add_style(waring_label, &style_waring_label_no_warning, 0); // 添加样式
    lv_obj_set_grid_cell(waring_label, LV_GRID_ALIGN_CENTER, 1, 2, LV_GRID_ALIGN_CENTER, 1, 1);
    // 设置心电图容器
    ecg_cont = lv_obj_create(cont);
    lv_obj_set_size(ecg_cont, 396, 190);
    lv_obj_set_style_pad_all(ecg_cont, 0, 0);
    lv_obj_set_style_radius(ecg_cont, 0, 0); // 去除圆角
    lv_obj_set_style_bg_color(ecg_cont, lv_color_make(0xE0, 0xFF, 0xFF), LV_STATE_DEFAULT);
    lv_obj_set_grid_cell(ecg_cont, LV_GRID_ALIGN_CENTER, 0, 3, LV_GRID_ALIGN_CENTER, 2, 1);
    lv_obj_clear_flag(ecg_cont, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_scrollbar_mode(ecg_cont, LV_SCROLLBAR_MODE_OFF); // 关闭滚动条
    // 设置心电图
    ecg_line = lv_line_create(ecg_cont);
    lv_obj_set_size(ecg_line, 396, 180); // 设置心电图大小
    lv_obj_add_style(ecg_line, &style_ecg_line, 0); // 添加样式
    lv_obj_align(ecg_line, LV_ALIGN_LEFT_MID, 0, 0); // 左对齐
    lv_line_set_y_invert(ecg_line, true); // Y轴反转
    // 设置心电图刷新指示器
    ecg_indicator = lv_obj_create(ecg_cont);
    lv_obj_add_style(ecg_indicator, &style_ecg_indicator, 0);
    lv_obj_set_size(ecg_indicator, 2, 190); // 宽2，高190
    lv_obj_set_pos(ecg_indicator, 0, 0);    // 初始位置
    lv_obj_add_flag(ecg_indicator, LV_OBJ_FLAG_HIDDEN); // 初始隐藏
    // 设置心电图线在最前端
    lv_obj_move_foreground(ecg_line);
}
/**
 * @brief 添加页面2的内容
 * @param cont 页面2的容器对象
 * @note 该函数清除之前的内容，设置滚动条模式，添加标签等UI元素
 * @return void
 */
static void add_content_to_page_2(lv_obj_t *cont)
{
    // 清除之前的内容
    lv_obj_clean(cont);
    // 清除滚动标志
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE); 
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    // 添加内容到子页面
    lv_obj_set_grid_dsc_array(cont, column_dsc_2, row_dsc_2);
    // 在 cont grid 布局中去除边距
    lv_obj_set_style_pad_column(cont, 0, 0);
    lv_obj_set_style_pad_row(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    // 设置设置头部容器
    setting_head_cont = lv_obj_create(cont);
    lv_obj_set_size(setting_head_cont, lv_pct(100), 60);
    lv_obj_set_style_bg_color(setting_head_cont, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(setting_head_cont, 0, 0);
    lv_obj_set_style_pad_all(setting_head_cont, 10, 0);
    lv_obj_set_style_radius(setting_head_cont, 0, 0); // 去除圆角
    lv_obj_clear_flag(setting_head_cont, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_grid_cell(setting_head_cont, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_CENTER, 0, 1);
    // 设置设置头部标签
    setting_head_label = lv_label_create(setting_head_cont);
    lv_label_set_text(setting_head_label, "系统设置"); // 设置标签文本
    lv_obj_set_align(setting_head_label, LV_ALIGN_LEFT_MID); // 左对齐
    lv_obj_add_style(setting_head_label, &usual_label_style, 0); // 添加样式
    // 设置重置按钮
    reset_btn = lv_btn_create(setting_head_cont);
    lv_obj_set_align(reset_btn, LV_ALIGN_RIGHT_MID); // 右对齐
    lv_obj_set_size(reset_btn, 80, 50); // 设置按钮大小
    reset_btn_label = lv_label_create(reset_btn);
    lv_label_set_text(reset_btn_label, "重置"); // 设置按钮文本
    lv_obj_center(reset_btn_label); // 标签居中
    lv_obj_add_style(reset_btn, &usual_label_style, 0); // 添加样式
    lv_obj_add_event_cb(reset_btn, reset_btn_event_cb, LV_EVENT_CLICKED, NULL); // 添加事件回调
    // 设置设置列表容器
    setting_cont = lv_obj_create(cont);
    lv_obj_set_size(setting_cont, lv_pct(100), 260);
    lv_obj_set_style_bg_color(setting_cont, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(setting_cont, 0, 0);
    lv_obj_set_style_pad_all(setting_cont, 0, 0);
    lv_obj_set_style_radius(setting_cont, 0, 0); // 去除圆角
    lv_obj_clear_flag(setting_cont, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_scrollbar_mode(setting_cont, LV_SCROLLBAR_MODE_AUTO); // 自动滚动条
    lv_obj_set_grid_cell(setting_cont, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_CENTER, 1, 1);
    // 设置设置列表
    setting_list = lv_obj_create(setting_cont);
    lv_obj_set_size(setting_list, lv_pct(100), 260);
    lv_obj_set_style_pad_all(setting_list, 10, 0);
    lv_obj_set_style_radius(setting_list, 0, 0); // 去除圆角
    lv_obj_set_style_bg_color(setting_list, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(setting_list, 0, 0);
    lv_obj_set_flex_flow(setting_list, LV_FLEX_FLOW_COLUMN); // 列表垂直布局
    lv_obj_set_flex_align(setting_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    // 添加设置项
    // 设置屏幕亮度
    brightness_item = lv_obj_create(setting_list);
    lv_obj_set_size(brightness_item, 380, 80);
    lv_obj_set_style_bg_color(brightness_item, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(brightness_item, 2, 0);
    lv_obj_set_style_pad_all(brightness_item, 10, 0);
    lv_obj_set_style_radius(brightness_item, 5, 0);
    lv_obj_clear_flag(brightness_item, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_flex_flow(brightness_item, LV_FLEX_FLOW_ROW); // 设置为横向流式布局
    lv_obj_set_flex_align(brightness_item, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    // 设置亮度标签
    brightness_label = lv_label_create(brightness_item);
    lv_label_set_text(brightness_label, "屏幕亮度"); // 设置标签文本
    lv_obj_set_align(brightness_label, LV_ALIGN_LEFT_MID); // 左对齐
    lv_obj_add_style(brightness_label, &usual_label_style, 0); // 添加样式
    // 设置亮度滑块
    brightness_bar = lv_slider_create(brightness_item);
    lv_obj_set_size(brightness_bar, 160, 20); // 设置滑块大小
    lv_slider_set_range(brightness_bar, BACKLIGHT_BRIGHTNESS_MIN, BACKLIGHT_BRIGHTNESS_MAX); // 设置滑块范围
    lv_slider_set_value(brightness_bar, BACKLIGHT_DEFAULT_BRIGHTNESS, LV_ANIM_OFF); // 设置初始值为80
    lv_obj_add_event_cb(brightness_bar, brightness_bar_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // 设置亮度值对象
    brightness_value_label = lv_label_create(brightness_item);
    lv_label_set_text_fmt(brightness_value_label, "%d%%", BACKLIGHT_DEFAULT_BRIGHTNESS); // 设置初始文本
    lv_obj_add_style(brightness_value_label, &usual_label_style, 0); // 添加样式
    // 设置高心率
    high_alarm_item = lv_obj_create(setting_list);
    lv_obj_set_size(high_alarm_item, 380, 80);
    lv_obj_set_style_bg_color(high_alarm_item, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(high_alarm_item, 2, 0);
    lv_obj_set_style_pad_all(high_alarm_item, 10, 0);
    lv_obj_set_style_radius(high_alarm_item, 5, 0);
    lv_obj_clear_flag(high_alarm_item, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_flex_flow(high_alarm_item, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(high_alarm_item, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    // 设置高心率标签
    high_alarm_label = lv_label_create(high_alarm_item);
    lv_label_set_text(high_alarm_label, "高心率阈值"); // 设置标签文本
    lv_obj_set_align(high_alarm_label, LV_ALIGN_LEFT_MID); // 左对齐
    lv_obj_add_style(high_alarm_label, &usual_label_style, 0); // 添加样式
    // 设置高心率滑块
    high_alarm_bar = lv_slider_create(high_alarm_item);
    lv_obj_set_size(high_alarm_bar, 160, 20); // 设置滑块大小
    lv_slider_set_range(high_alarm_bar, min_high_alarm, max_high_alarm); // 设置滑块范围
    lv_slider_set_value(high_alarm_bar, HIGH_ALARM_DEFAULT, LV_ANIM_OFF); // 设置初始值为180
    lv_obj_add_event_cb(high_alarm_bar, high_alarm_bar_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // 设置高心率值对象
    high_alarm_value_label = lv_label_create(high_alarm_item);
    lv_label_set_text_fmt(high_alarm_value_label, "%d bpm", HIGH_ALARM_DEFAULT); // 设置初始文本
    lv_obj_add_style(high_alarm_value_label, &usual_label_style, 0); // 添加样式
    // 设置低心率
    low_alarm_item = lv_obj_create(setting_list);
    lv_obj_set_size(low_alarm_item, 380, 80);
    lv_obj_set_style_bg_color(low_alarm_item, lv_color_make(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_border_width(low_alarm_item, 2, 0);
    lv_obj_set_style_pad_all(low_alarm_item, 10, 0);
    lv_obj_set_style_radius(low_alarm_item, 5, 0);
    lv_obj_clear_flag(low_alarm_item, LV_OBJ_FLAG_SCROLLABLE); // 清除滚动标志
    lv_obj_set_flex_flow(low_alarm_item, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(low_alarm_item, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    // 设置低心率标签
    low_alarm_label = lv_label_create(low_alarm_item);
    lv_label_set_text(low_alarm_label, "低心率阈值"); // 设置标签文本
    lv_obj_set_align(low_alarm_label, LV_ALIGN_LEFT_MID); // 左对齐
    lv_obj_add_style(low_alarm_label, &usual_label_style, 0); // 添加样式
    // 设置低心率滑块
    low_alarm_bar = lv_slider_create(low_alarm_item);
    lv_obj_set_size(low_alarm_bar, 160, 20); // 设置滑块大小
    // 展示
    lv_slider_set_range(low_alarm_bar, min_low_alarm, 80); // 设置滑块范围
    //lv_slider_set_range(low_alarm_bar, min_low_alarm, max_low_alarm); // 设置滑块范围
    lv_slider_set_value(low_alarm_bar, LOW_ALARM_DEFAULT, LV_ANIM_OFF); // 设置初始值为40
    lv_obj_add_event_cb(low_alarm_bar, low_alarm_bar_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // 设置低心率值对象
    low_alarm_value_label = lv_label_create(low_alarm_item);
    lv_label_set_text_fmt(low_alarm_value_label, "%d bpm", LOW_ALARM_DEFAULT); // 设置初始文本
    lv_obj_add_style(low_alarm_value_label, &usual_label_style, 0); // 添加样式

}
/**
 * @brief 初始化 LVGL 样式
 * @param None
 * @return void
 * @note 该函数初始化了各种样式，包括屏幕样式、菜单按钮样式、标签样式等
 */
static void lvgl_style_init()
{
    // 初始化屏幕样式
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, lv_color_black());                 // 设置背景色为黑色
    lv_style_set_border_width(&style_screen, 0);                            // 设置边框宽度为0
    lv_style_set_radius(&style_screen, 0);                                  // 设置圆角半径为0
    lv_style_set_shadow_width(&style_screen, 0);                            // 去除阴影

    // 初始化菜单按钮样式
    lv_style_init(&style_menu_btn);
    lv_style_set_bg_color(&style_menu_btn, lv_color_hex(0xFFFFFF));         // 白色背景
    lv_style_set_text_color(&style_menu_btn, lv_color_hex(0x000000));       // 黑色文字
    lv_style_set_text_font(&style_menu_btn, &JiDianHei_18);                 // 字体设置
    lv_style_set_text_align(&style_menu_btn, LV_TEXT_ALIGN_CENTER);         // 文本居中对齐
    lv_style_set_radius(&style_menu_btn, 0);                                // 去除圆角
    lv_style_set_shadow_width(&style_menu_btn, 0);                          // 去除阴影

    // 初始化普通标签样式
    lv_style_init(&usual_label_style);
    lv_style_set_text_font(&usual_label_style, &JiDianHei_18);              // 字体设置
    lv_style_set_text_align(&usual_label_style, LV_TEXT_ALIGN_CENTER);      // 文本居中对齐

    // 初始化警告标签样式
    lv_style_init(&style_waring_label);
    lv_style_set_text_color(&style_waring_label, lv_color_hex(0xFF0000));   // 红色文字
    lv_style_set_text_font(&style_waring_label, &JiDianHei_18);             // 字体设置
    lv_style_set_text_align(&style_waring_label, LV_TEXT_ALIGN_CENTER);     // 文本居中对齐

    // 初始化无警告标签样式
    lv_style_init(&style_waring_label_no_warning);
    lv_style_set_text_color(&style_waring_label_no_warning, lv_color_hex(0x00EE00)); // 绿色文字
    lv_style_set_text_font(&style_waring_label_no_warning, &JiDianHei_18);          // 字体设置
    lv_style_set_text_align(&style_waring_label_no_warning, LV_TEXT_ALIGN_CENTER); // 文本居中对齐

    // 初始化心率标签样式
    lv_style_init(&style_hr_label);
    lv_style_set_text_color(&style_hr_label, lv_color_hex(0x000000));       // 黑色文字
    lv_style_set_text_font(&style_hr_label, &JiDianHei_18);        // 字体设置
    lv_style_set_text_align(&style_hr_label, LV_TEXT_ALIGN_CENTER);         // 文本居中对齐

    // 初始化ECG开始按钮样式
    lv_style_init(&style_ecg_start);
    lv_style_set_bg_color(&style_ecg_start, lv_color_hex(0x00EE00));        // 绿色背景
    lv_style_set_text_color(&style_ecg_start, lv_color_hex(0xFFFFFF));      // 白色文字
    lv_style_set_shadow_width(&style_ecg_start, 0);// 去除阴影

    // 初始化ECG停止按钮样式
    lv_style_init(&style_ecg_stop);
    lv_style_set_bg_color(&style_ecg_stop, lv_color_hex(0xFF4040));         // 红色背景
    lv_style_set_text_color(&style_ecg_stop, lv_color_hex(0xFFFFFF));       // 白色文字
    lv_style_set_shadow_width(&style_ecg_stop, 0);// 去除阴影

    // 初始化传输开始按钮样式
    lv_style_init(&style_transmit_start);
    lv_style_set_bg_color(&style_transmit_start, lv_color_hex(0x0080FF));     // 蓝色背景
    lv_style_set_text_color(&style_transmit_start, lv_color_hex(0xFFFFFF));   // 白色文字
    lv_style_set_shadow_width(&style_transmit_start, 0);// 去除阴影

    // 初始化传输停止按钮样式
    lv_style_init(&style_tarnsmit_stop);
    lv_style_set_bg_color(&style_tarnsmit_stop, lv_color_hex(0xFF4040));      // 红色背景
    lv_style_set_text_color(&style_transmit_start, lv_color_hex(0xFFFFFF));   // 白色文字
    lv_style_set_shadow_width(&style_transmit_start, 0);// 去除阴影

    // 初始化ECG线条样式
    lv_style_init(&style_ecg_line);
    lv_style_set_line_color(&style_ecg_line, lv_color_hex(0xF00000));       // 红色线条
    lv_style_set_line_width(&style_ecg_line, 2);                            // 线条宽度
    lv_style_set_line_rounded(&style_ecg_line, false);                      // 线条圆角
    lv_style_set_shadow_width(&style_ecg_line, 0);                          // 去除阴影
    lv_style_set_pad_all(&style_ecg_line, 0); // 去除内边距

    // 初始化ECG指示器样式
    lv_style_init(&style_ecg_indicator);
    lv_style_set_bg_color(&style_ecg_indicator, lv_color_hex(0x0080FF));         // 蓝色背景
    lv_style_set_border_width(&style_ecg_indicator, 0);                     // 去除边框
    lv_style_set_radius(&style_ecg_indicator, 2);                           // 设置圆角半径
    lv_style_set_radius(&style_ecg_indicator, 0);                           // 无圆角
    lv_style_set_width(&style_ecg_indicator, 2);                            // 线宽2像素
    lv_style_set_height(&style_ecg_indicator, 180);                         // 高度与ecg_cont一致
    lv_style_set_pad_all(&style_ecg_indicator, 0);

}
/**
 * @brief 菜单按钮点击事件回调函数
 * @param e 事件对象
 * @return void
 * @note 该函数处理菜单按钮的点击事件，移动指示线并切换页面显示
 *       它会获取按钮的坐标和大小，计算目标位置，并使用动画移动指示线到目标位置
 */
static void menu_btn_event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    ESP_LOGI(TAG_LVGL, "菜单按钮点击事件: %s", lv_label_get_text(lv_obj_get_child(btn, 0)));
    // 获取按钮的坐标和大小
    lv_coord_t btn_y = lv_obj_get_y(btn);
    lv_coord_t indi_y = lv_obj_get_y(indicator);
    lv_coord_t target_y = btn_y + (menu_btn_height - 80) / 2;
    lv_obj_move_foreground(indicator); // 保证在最上层
    // 播放动画
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, indicator);
    lv_anim_set_exec_cb(&anim, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_values(&anim, indi_y, target_y);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_in_out);
    lv_anim_set_time(&anim, 300);
    lv_anim_set_delay(&anim, 0);
    lv_anim_start(&anim);
    // 渲染页面
    if(btn == menu_btn_1)
    {
        lv_obj_clear_flag(page_1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_2, LV_OBJ_FLAG_HIDDEN);
    }
    else if (btn == menu_btn_2)
    {
        lv_obj_clear_flag(page_2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_1, LV_OBJ_FLAG_HIDDEN);
    }
}
/**
 * @brief ECG 按钮点击事件回调函数
 * @param e 事件对象
 * @return void
 * @note 该函数处理ECG按钮的点击事件，根据当前采样和传输状态切换按钮样式和文本，并启动或停止采样任务
 */
static void ecg_btn_event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    ESP_LOGI(TAG_LVGL, "ECG 按钮点击事件: %s", lv_label_get_text(lv_obj_get_child(btn, 0)));
    if(ecg_state.is_sampling)
    {
        // 如果正在采样
        if(ecg_state.is_transmitting)
        {
            // 如果正在采样并且正在传输
            // 先停止传输
            lv_event_send(transmit_btn, LV_EVENT_CLICKED, NULL);
            // 再停止采样
            set_warning_label("停止采集...", false); // 设置警告标签文本和样式
            lv_obj_add_style(btn, &style_ecg_start, LV_STATE_DEFAULT);
            lv_label_set_text(lv_obj_get_child(btn, 0), "开始采集");
            // 停止采样
            // 给任务发送通知使其退出
            ecg_state.is_sampling = !ecg_state.is_sampling; // 切换采样状态
            if (ecg_sample_task_handle != NULL) {
               vTaskNotifyGiveFromISR(ecg_sample_task_handle, NULL); // 发送通知使任务退出
            }
        }
        else
        {
            // 如果正在采样但未传输
            set_warning_label("停止采集...", false); // 设置警告标签文本和样式
            lv_obj_add_style(btn, &style_ecg_start, LV_STATE_DEFAULT);
            lv_label_set_text(lv_obj_get_child(btn, 0), "开始采集");
            // 停止采样
            ecg_state.is_sampling = !ecg_state.is_sampling; // 切换采样状态
            if (ecg_sample_task_handle != NULL) {
               vTaskNotifyGiveFromISR(ecg_sample_task_handle, NULL); // 发送通知使任务退出
            }
        }    
    }
    else 
    {
        // 如果未采样
        read_adc_chip_state(); // 读取ADC芯片状态
        if(adc_chip_state.is_LON_LOW == false && adc_chip_state.is_LOP_LOW == false)
        {
            // 如果导联未连接
            set_warning_label("请先连接导联", true); // 设置警告标签文本和样式
            return; // 不执行采样逻辑
        }
        else if(adc_chip_state.is_LON_LOW == false && adc_chip_state.is_LOP_LOW == true)
        {
            // 如果导联连接不正确
            set_warning_label("导联连接不正确", true); // 设置警告标签文本和样式
            return; // 不执行采样逻辑
        }
        else if(adc_chip_state.is_LON_LOW == true && adc_chip_state.is_LOP_LOW == false)
        {
            // 如果导联连接不正确
            set_warning_label("导联连接不正确", true); // 设置警告标签文本和样式
            return; // 不执行采样逻辑
        }
        else if(adc_chip_state.is_LON_LOW == true && adc_chip_state.is_LOP_LOW == true)
        {
            // 如果导联连接正确
            set_warning_label("启动采集...", false); // 设置警告标签文本和样式
            lv_obj_add_style(btn, &style_ecg_stop, LV_STATE_DEFAULT);
            lv_label_set_text(lv_obj_get_child(btn, 0), "停止采集");
            // 开始采样
            ecg_state.is_sampling = !ecg_state.is_sampling; // 切换采样状态
            lv_label_set_text(hr_label, "心率: 计算中...");
        }
    
    }
    
    
}
/**
 * @brief 传输按钮点击事件回调函数
 * @param e 事件对象
 * @return void
 * @note 该函数处理传输按钮的点击事件，根据当前传输状态切换按钮样式和文本，并开始或停止传输
 */
static void transmit_btn_event_cb(lv_event_t *e)
{
    lv_obj_t *transmit_btn = lv_event_get_target(e);
    ESP_LOGI(TAG_LVGL, "传输按钮点击事件: %s", lv_label_get_text(lv_obj_get_child(transmit_btn, 0)));
    if(ecg_state.is_transmitting) 
    {
        // 如果正在传输
        // 停止传输
        set_warning_label("停止传输...", false); // 设置警告标签文本和样式
        lv_obj_add_style(transmit_btn, &style_transmit_start, LV_STATE_DEFAULT);
        lv_label_set_text(lv_obj_get_child(transmit_btn, 0), "串口传输");
        ecg_state.is_transmitting = !ecg_state.is_transmitting; // 切换传输状态
    }
    else
    {   
        // 如果未传输
        // 开始传输  
        if(ecg_state.is_sampling)
        {
            // 如果正在采样
            set_warning_label("开始传输...", false); // 设置警告标签文本和样式
            lv_obj_add_style(transmit_btn, &style_tarnsmit_stop, LV_STATE_DEFAULT);
            lv_label_set_text(lv_obj_get_child(transmit_btn, 0), "停止传输");
            ecg_state.is_transmitting = !ecg_state.is_transmitting; // 切换传输状态
        }
        else
        {
            // 如果未采样
            // 提示用户先采样
            set_warning_label("请先开始采集", true); // 设置警告标签文本和样式
            // 切换按钮样式
            lv_obj_add_style(transmit_btn, &style_transmit_start, LV_STATE_DEFAULT);
            lv_label_set_text(lv_obj_get_child(transmit_btn, 0), "串口传输");
            return; // 不执行传输逻辑
        }
    }
    
}
/**
 * @brief 重设警告标签文本和样式
 * @param text 要设置的文本
 * @param is_warning 是否为警告状态
 * @return void
 */
static void set_warning_label(const char *text, const bool is_warning)
{
    if(is_warning)
    {
        lv_label_set_text(waring_label, text);
        lv_obj_add_style(waring_label, &style_waring_label, LV_STATE_DEFAULT); // 设置警告标签样式为有警告
        if (warning_label_Task_handle == NULL) {
            // 如果警告标签任务未创建，则创建
            xTaskCreatePinnedToCore(warning_label_Task, "警告标签任务", 2048, NULL, 3, &warning_label_Task_handle, 1);
        }
    }
    else{
        lv_label_set_text(waring_label, text);
        lv_obj_add_style(waring_label, &style_waring_label_no_warning, LV_STATE_DEFAULT); // 设置警告标签样式为无警告
    }
}
/**
 * @brief 重置按钮点击事件回调函数
 * @param e 事件对象
 * @return void
 */
static void reset_btn_event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    ESP_LOGI(TAG_LVGL, "重置按钮点击事件: %s", lv_label_get_text(lv_obj_get_child(btn, 0)));
    // 重置设置
    // 重置背光亮度
    lv_slider_set_value(brightness_bar, BACKLIGHT_DEFAULT_BRIGHTNESS, LV_ANIM_OFF); // 设置默认亮度
    lv_label_set_text_fmt(brightness_value_label, "%d%%", BACKLIGHT_DEFAULT_BRIGHTNESS); // 更新亮度值标签
    // 重置高心率报警
    lv_slider_set_value(high_alarm_bar, HIGH_ALARM_DEFAULT, LV_ANIM_OFF); // 设置默认高心率报警值
    lv_label_set_text_fmt(high_alarm_value_label, "%d bpm", HIGH_ALARM_DEFAULT); // 更新高心率报警值标签
    // 重置低心率报警
    lv_slider_set_value(low_alarm_bar, LOW_ALARM_DEFAULT, LV_ANIM_OFF); // 设置默认低心率报警值
    lv_label_set_text_fmt(low_alarm_value_label, "%d bpm", LOW_ALARM_DEFAULT); // 更新低心率报警值标签
}
/**
 * @brief 设置背光亮度滑块事件回调函数
 * @param e 事件对象
 * @return void
 */
static void brightness_bar_event_cb(lv_event_t *e)
{
    lv_obj_t *bar = lv_event_get_target(e);
    int value = lv_slider_get_value(bar); // 获取滑块值
    //ESP_LOGI(TAG_LVGL, "设置背光亮度: %d%%", value);
    // 设置背光亮度
    if (value >= BACKLIGHT_BRIGHTNESS_MIN && value <= BACKLIGHT_BRIGHTNESS_MAX) {
        display_brightness_set(value); // 设置背光亮度
        lv_label_set_text_fmt(brightness_value_label, "%d%%", value); // 更新标签文本
    } else {
        ESP_LOGW(TAG_LVGL, "无效的背光亮度值: %d", value);
    }
}
/**
 * @brief 高心率报警滑块事件回调函数
 * @param e 事件对象
 * @return void
 */
static void high_alarm_bar_event_cb(lv_event_t *e)
{
    lv_obj_t *bar = lv_event_get_target(e);
    int value = lv_slider_get_value(bar); // 获取滑块值
    //ESP_LOGI(TAG_LVGL, "设置高心率报警: %d bpm", value);
    // 设置高心率报警阈值
    if (value >= min_high_alarm && value <= max_high_alarm) {
        high_alarm = value; // 更新高心率报警阈值
        lv_label_set_text_fmt(high_alarm_value_label, "%d bpm", value); // 更新标签文本
    } else {
        ESP_LOGW(TAG_LVGL, "无效的高心率报警值: %d", value);
    }
}
/**
 * @brief 低心率报警滑块事件回调函数
 * @param e 事件对象
 * @return void
 */
static void low_alarm_bar_event_cb(lv_event_t *e)
{
    lv_obj_t *bar = lv_event_get_target(e);
    int value = lv_slider_get_value(bar); // 获取滑块值
    //ESP_LOGI(TAG_LVGL, "设置低心率报警: %d bpm", value);
    // 设置低心率报警阈值
    // if (value >= min_low_alarm && value <= max_low_alarm) {
    //     low_alarm = value; // 更新低心率报警阈值
    //     lv_label_set_text_fmt(low_alarm_value_label, "%d bpm", value); // 更新标签文本
    // } else {
    //     ESP_LOGW(TAG_LVGL, "无效的低心率报警值: %d", value);
    // }
    // 展示
    if (value >= min_low_alarm) {
        low_alarm = value; // 更新低心率报警阈值
        lv_label_set_text_fmt(low_alarm_value_label, "%d bpm", value); // 更新标签文本
    }
}
/**
 * @brief 将 uint16_t 转换为 int16_t
 * @param x 要转换的 uint16_t 值
 * @return int16_t 转换后的 int16_t 值
 * @note 该函数将 uint16_t 值转换为 int16_t 值
 */
int16_t uint16_to_int16(uint16_t x) 
{
    return (int16_t)(x - 32768);
}
/**
 * @brief 将 int16_t 转换为 uint16_t
 * @param y 要转换的 int16_t 值
 * @return uint16_t 转换后的 uint16_t 值
 * @note 该函数将 int16_t 值转换为 uint16_t 值
 */
uint16_t int16_to_uint16(int16_t y) 
{
    return (uint16_t)(y + 32768);
}
/*
    任务函数
*/
/**
 * @brief 蜂鸣器警告任务
 * 此任务负责在心率异常时发出蜂鸣器警告
 */
static void buzzer_warning_Task()
{
    // 注册当前任务到看门狗
    esp_task_wdt_add(NULL);  // 添加到任务看门狗监控
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒执行一次
        if(is_alarm && ecg_state.is_sampling == true)
        {
            ESP_LOGW(TAG_BUZZER, "心率异常: %.2f BPM", heart_rate); // 打印警告日志
            if (waring_label != NULL) // 如果警告标签存在
            {
                set_warning_label("心率异常", true); // 设置警告标签文本和状态
            }
            for(uint8_t i = 0; i < 3; i++) // 循环3次
            {
                buzzer_beep_on(2000); // 打开蜂鸣器
                vTaskDelay(pdMS_TO_TICKS(100)); // 响100ms
                buzzer_beep_off(); // 关闭蜂鸣器
                vTaskDelay(pdMS_TO_TICKS(500)); // 停500ms

            }
        }
        else if(!is_alarm && ecg_state.is_sampling == true) // 如果没有报警且正在采样
        {
            // 如果没有报警，延迟1秒后继续循环
            if (waring_label != NULL) // 如果警告标签存在
            {
                set_warning_label("心率正常", false); // 设置警告标签文本和状态
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        esp_task_wdt_reset(); // 重置任务看门狗
    }
    // 蜂鸣器警告结束后删除任务
    if (buzzer_warning_Task_handle != NULL) // 如果任务句柄存在
    {
        // 从看门狗移除并删除任务
        esp_task_wdt_delete(NULL);
        buzzer_warning_Task_handle = NULL; // 清空任务句柄
        vTaskDelete(NULL); // 删除当前任务
    }
}
/**
 * @brief 警告标签任务
 * 此任务负责闪烁警告标签以提示用户
 */
static void warning_label_Task()
{
    esp_task_wdt_add(NULL);  // 添加到任务看门狗监控
    for(uint8_t i = 0; i < 3; i++) // 循环5次
    {
        if (waring_label != NULL) // 如果警告标签存在
        {
            // 闪烁警告标签
            lv_obj_add_flag(waring_label, LV_OBJ_FLAG_HIDDEN); // 隐藏标签
            vTaskDelay(pdMS_TO_TICKS(300)); // 延迟300毫秒
            lv_obj_clear_flag(waring_label, LV_OBJ_FLAG_HIDDEN); // 显示标签
            vTaskDelay(pdMS_TO_TICKS(700)); // 延迟700毫秒
        }
        esp_task_wdt_reset();
    }
    // 闪烁结束后删除任务
    if (warning_label_Task_handle != NULL) // 如果任务句柄存在
    {
        // 从看门狗移除并删除任务
        esp_task_wdt_delete(NULL);
        warning_label_Task_handle = NULL; // 清空任务句柄
        vTaskDelete(NULL); // 删除当前任务
    }
}
/**
 * @brief ECG采样任务
 * 此任务负责从ADC读取ECG数据并处理
 */
static void ecg_sample_Task()
{
    esp_task_wdt_add(NULL);                         // 添加到任务看门狗监控
    esp_err_t ret;                                  // 用于存储函数返回值
    uint32_t ret_num = 0;                           // 实际读取到的数据长度
    uint8_t result[READ_LEN] = {0};                 // DMA数据缓冲区
    // 在ecg_sample_Task开头添加
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &adc_handle);// 初始化ADC连续模式
    // 注册事件回调
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conviuous_done_cb,      // 绑定转换完成回调
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL)); 
    // 设置控制引脚低电平（启动ADC）
    ESP_ERROR_CHECK(gpio_set_level(ADC_CONTROL_GPIO_NUM, 1)); 
    // 重置ECG指示器位置
    lv_obj_clear_flag(ecg_indicator, LV_OBJ_FLAG_HIDDEN); // 显示ECG指示器
    lv_obj_set_x(ecg_indicator, 0); 
    // 清空ECG曲线数据
    memset(ecg_points, 0, sizeof(ecg_points)); // 清空ECG数据点数组
    ecg_point_index = 0; // 重置ECG点索引
    lv_line_set_points(ecg_line, ecg_points, 0); // 清空ECG曲线
    // 初始化FIR滤波器
    dsps_fird_init_s16(&fir, fir_coeffs, fir_delay, FIR_LEN, DOWNSAMPLE_FACTOR, 0, 0); 
    // 初始化Pan-Tompkins算法
    panTompkins_init();
    // 启动ADC连续模式 
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle)); 
    ESP_LOGI(TAG_ADC, "正在启动连续adc采样...");
    /* 等待回调函数通知数据就绪 */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);        // 阻塞等待通知（直到有数据可读）
    // // 采样率统计相关变量（测量实际采样率使用）
    // uint32_t sample_count = 0;
    // int64_t last_time_us = esp_timer_get_time();
    /* 循环读取DMA数据 */
    int16_t input[DOWNSAMPLE_FACTOR] = {0};
    int16_t output = 0; // 输出变量初始化
    while (ecg_state.is_sampling) // 如果正在采样
    {
        ret = adc_continuous_read(adc_handle, result, READ_LEN, &ret_num, 0);
        if (ret == ESP_OK) 
        {
            // 统计采样点数 (测量实际采样率使用)
            // uint32_t points = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
            // sample_count += points;
            uint8_t draw_count = 0;
            uint16_t draw_buf[3] = {0};
            
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) 
            {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint16_t chan_num = ADC_GET_CHANNEL(p);     // 提取通道号
                uint16_t data = ADC_GET_DATA(p);            // 提取采样值
                // 通过通道号验证数据有效性（超过最大通道号则数据无效）
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT)) 
                {
                    input[downsample_count] = uint16_to_int16(data); // 将uint16转换为int16并存入输入数组
                    downsample_count++;
                    if (downsample_count == DOWNSAMPLE_FACTOR - 1) // 达到降采样因子
                    {
                        
                        int32_t actual = dsps_fird_s16_ae32(&fir, input, &output, 1); // FIR滤波处理
                        downsample_count = 0;
                        memset(input, 0, sizeof(input)); // 清空输入数组
                        if(actual > 0)
                        {
                            uint16_t result_data = int16_to_uint16(output); // 将int16转换为uint16
                            if(ecg_state.is_transmitting) // 如果正在传输数据
                            {   
                            // 这里可以添加数据传输逻辑
                            // 例如将数据发送到显示屏或其他设备
                                ESP_LOGI(TAG_ECG, "采样值: %d", result_data);
                            }
                            /* pT算法计算心率 */
                            // 送入Pan-Tompkins算法
                            panTompkins_process(result_data); // 处理当前采样值
                            // 检查是否检测到R峰
                            uint32_t detected_idx;
                            if (panTompkins_get_detection(&detected_idx)) 
                            {
                            heart_rate = panTompkins_get_heart_rate();
                            if(qrs_count < 10)
                            {
                                qrs_count++; // 增加QRS波计数
                                // 如果QRS波计数小于10，则不显示心率
                                lv_label_set_text(hr_label, "心率: 计算中...");
                            }
                            else
                            {
                                char hr_str[32];
                                snprintf(hr_str, sizeof(hr_str), "心率: %.0f bpm", heart_rate);
                                // 更新心率标签
                                if (hr_label == NULL) 
                                {
                                    ESP_LOGE(TAG_ECG, "心率标签未初始化!");
                                    continue; // 如果标签未初始化则跳过
                                }
                                else
                                {
                                    
                                    lv_label_set_text(hr_label, hr_str); // 更新心率标签文本
                                }
                                if(heart_rate > high_alarm || heart_rate < low_alarm) // 如果心率超过报警阈值
                                {
                                    is_alarm = true; // 设置报警状态
                                }
                                else
                                {
                                    is_alarm = false; // 重置报警状态
                                }
                                // 也可以保存或输出心率数据
                                // ESP_LOGI(TAG_ECG, "R峰检出! 样本点: %lu, 当前心率: %.1f BPM", detected_idx, current_hr);
                            } 
                            }
                            /* 绘制心电图 */
                            // 线性映射到1~179
                            //uint16_t y = -30 + 1 + (uint16_t)data * 178 / 4095;
                            uint16_t y = 1 + (uint16_t)((result_data - 11000) * (179 - 1) / (14500 - 11500)) - 30;
                            if (y > 179) y = 179;
                            if (y < 1) y = 1;
                            // 收集3个点
                            draw_buf[draw_count++] = y;
                            if (draw_count == 3) 
                            {
                            // 添加到ecg_points
                            uint16_t sum = draw_buf[0] + draw_buf[1] + draw_buf[2];
                            ecg_points[ecg_point_index].x = ecg_point_index;
                            ecg_points[ecg_point_index].y = sum / 3; // 平均值
                            ecg_point_index++;
                            if (ecg_point_index >= 396) ecg_point_index = 0; // 循环显示
                            // 刷新LVGL曲线
                            // 计算当前有效点数
                            uint16_t valid_points = ecg_point_index < 396 ? ecg_point_index : 396;
                            lv_line_set_points(ecg_line, ecg_points, valid_points);
                            lv_obj_set_x(ecg_indicator, ecg_point_index); // 横坐标随ecg索引移动
                            draw_count = 0;
                            }
                        }
                        else
                        {
                            ESP_LOGE(TAG_ADC, "FIR滤波处理失败"); // 输出错误日志
                        }
                        output = 0; // 重置输出变量
                    }
                } 
                else 
                {
                    ESP_LOGW(TAG_ADC, "无效通道号: %d", chan_num); // 输出警告日志
                }
            }
            // 每秒统计一次实际采样率（用于统计实际采样率）
            // int64_t now_us = esp_timer_get_time();
            // if (now_us - last_time_us >= 1000000) {
            //     ESP_LOGI(TAG_ADC, "实际采样率: %ld Hz", sample_count);
            //      sample_count = 0;
            //      last_time_us = now_us;
            // }
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));  // 防止看门狗超时
        }
        else if (ret == ESP_ERR_TIMEOUT) 
        {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue; // 继续读取数据
        }
    }

    /* 停止并释放ADC资源 */
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
    ESP_ERROR_CHECK(gpio_set_level(ADC_CONTROL_GPIO_NUM, 0)); // 关闭ADC

    lv_label_set_text(hr_label, "心率: -- bpm"); // 重置心率标签
    qrs_count = 0; // 重置QRS波计数
    ESP_LOGI(TAG_ECG, "ECG采样任务已停止");
    // 从看门狗移除并删除任务
    esp_task_wdt_delete(NULL);
    ecg_sample_task_handle = NULL;
    vTaskDelete(NULL);
}
/*
    以上是函数定义
*/

