
/**
 * @file    ecg_hardware.c
 * @brief   ECG 硬件抽象层: GPIO、SPI、ADC、Display、Touch、Buzzer、Backlight
 * @note    所有硬件操作通过此模块统一管理，不依赖 UI 或业务逻辑
 */

#include "ecg_hardware.h"
#include "ecg_config.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_lcd_ili9488.h"

static const char *TAG = "HW";

/* ---- 内部状态 ---- */
static ecg_hw_state_t       hw_state;
static adc_continuous_handle_t adc_handle;
static esp_lcd_panel_io_handle_t lcd_io;
static esp_lcd_panel_handle_t    lcd_panel;
static spi_device_handle_t   touch_spi;
static lv_disp_drv_t        *g_disp_drv;   // LVGL flush 通知用

/* ---- 内部辅助: LVGL flush ready 回调 ---- */
static bool IRAM_ATTR notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io,
        esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    if (g_disp_drv) lv_disp_flush_ready(g_disp_drv);
    return false;
}

/* ============================================================
 * GPIO
 * ============================================================ */
void hw_gpio_init(void)
{
    gpio_config_t cfg_in = {
        .pin_bit_mask = (1ULL << ADC_LON_GPIO) | (1ULL << ADC_LOP_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_in);

    gpio_config_t cfg_out = {
        .pin_bit_mask = (1ULL << ADC_CONTROL_GPIO) | (1ULL << BUZZER_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_out);
    gpio_set_level(ADC_CONTROL_GPIO, 0);
    ESP_LOGI(TAG, "GPIO 初始化完成");
}

/* ============================================================
 * SPI
 * ============================================================ */
void hw_spi_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = DISPLAY_SPI_MOSI,
        .miso_io_num = DISPLAY_SPI_MISO,
        .sclk_io_num = DISPLAY_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = DISPLAY_MAX_TRANSFER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    bus.mosi_io_num = TOUCH_SPI_MOSI;
    bus.miso_io_num = TOUCH_SPI_MISO;
    bus.sclk_io_num = TOUCH_SPI_CLK;
    bus.max_transfer_sz = TOUCH_MAX_TRANSFER;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI 总线初始化完成");
}

/* ============================================================
 * ADC
 * ============================================================ */
void hw_adc_init(adc_channel_t channel)
{
    adc_continuous_handle_cfg_t hcfg = {
        .max_store_buf_size = ADC_MAX_STORE_BUF_SIZE,
        .conv_frame_size    = ADC_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&hcfg, &adc_handle));

    adc_digi_pattern_config_t pat = {
        .atten     = ADC_ATTEN,
        .channel   = channel & 0x7,
        .unit      = ADC_UNIT,
        .bit_width = ADC_BIT_WIDTH,
    };

    adc_continuous_config_t dcfg = {
        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_OUTPUT_TYPE,
        .pattern_num    = 1,
        .adc_pattern    = &pat,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dcfg));
    ESP_LOGI(TAG, "ADC 初始化完成 (freq=%d Hz, ch=%d)", ADC_SAMPLE_FREQ_HZ, channel);
}

void hw_adc_register_done_cb(adc_continuous_evt_cbs_t *cbs)
{
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, cbs, NULL));
}

void hw_adc_start(void)
{
    gpio_set_level(ADC_CONTROL_GPIO, 1);
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    hw_state.is_sampling = true;
    ESP_LOGI(TAG, "ADC 采样已启动");
}

void hw_adc_stop(void)
{
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    gpio_set_level(ADC_CONTROL_GPIO, 0);
    hw_state.is_sampling = false;
    ESP_LOGI(TAG, "ADC 采样已停止");
}

void hw_adc_deinit(void)
{
    if (hw_state.is_sampling) hw_adc_stop();
    if (adc_handle) {
        adc_continuous_deinit(adc_handle);
        adc_handle = NULL;
    }
}

bool hw_adc_read(uint8_t *buf, uint32_t len, uint32_t *out)
{
    return (adc_continuous_read(adc_handle, buf, len, out, 0) == ESP_OK);
}

void hw_read_adc_status(bool *lon, bool *lop)
{
    if (lon) *lon = !gpio_get_level(ADC_LON_GPIO);
    if (lop) *lop = !gpio_get_level(ADC_LOP_GPIO);
}

adc_continuous_handle_t hw_get_adc_handle(void) { return adc_handle; }

/* ============================================================
 * 蜂鸣器
 * ============================================================ */
void hw_buzzer_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = BUZZER_LEDC_MODE, .timer_num = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_LEDC_DUTY_RES, .freq_hz = BUZZER_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t ch = {
        .gpio_num = BUZZER_GPIO, .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL, .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    ESP_LOGI(TAG, "蜂鸣器初始化完成");
}

void hw_buzzer_on(uint32_t freq)
{
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, freq);
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL,
                  (1 << BUZZER_LEDC_DUTY_RES) / 2);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

void hw_buzzer_off(void)
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

/* ============================================================
 * 显示屏
 * ============================================================ */
void hw_display_init(void)
{
    esp_lcd_panel_io_spi_config_t io = {
        .cs_gpio_num = DISPLAY_SPI_CS, .dc_gpio_num = DISPLAY_DC,
        .spi_mode = 0, .pclk_hz = DISPLAY_REFRESH_HZ,
        .trans_queue_depth = DISPLAY_SPI_QUEUE_LEN,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .lcd_cmd_bits = DISPLAY_CMD_BITS, .lcd_param_bits = DISPLAY_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                              &io, &lcd_io));

    esp_lcd_panel_dev_config_t p = {
        .reset_gpio_num = DISPLAY_RESET,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian    = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 18,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(lcd_io, &p, LV_BUFFER_SIZE, &lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_panel, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));
    ESP_LOGI(TAG, "显示屏 ILI9488 初始化完成 (%dx%d)", DISPLAY_H_RES, DISPLAY_V_RES);
}

esp_lcd_panel_handle_t hw_get_lcd_panel(void) { return lcd_panel; }

/* ============================================================
 * 背光
 * ============================================================ */
void hw_backlight_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = BACKLIGHT_LEDC_MODE, .timer_num = BACKLIGHT_LEDC_TIMER,
        .duty_resolution = BACKLIGHT_LEDC_TIMER_RES,
        .freq_hz = BACKLIGHT_LEDC_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t ch = {
        .gpio_num = DISPLAY_BACKLIGHT, .speed_mode = BACKLIGHT_LEDC_MODE,
        .channel = BACKLIGHT_LEDC_CHANNEL, .timer_sel = BACKLIGHT_LEDC_TIMER,
        .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    ESP_LOGI(TAG, "背光 PWM 初始化完成");
}

void hw_backlight_set(uint8_t pct)
{
    if (pct > 100) pct = 100;
    uint32_t duty = (1023 * pct) / 100;
    ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL, duty);
    ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL);
}

/* ============================================================
 * 触摸
 * ============================================================ */
void hw_touch_init(void)
{
    spi_device_interface_config_t d = {
        .clock_speed_hz = TOUCH_SPI_FREQ_HZ, .mode = 0,
        .spics_io_num = TOUCH_SPI_CS, .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &d, &touch_spi));
    ESP_LOGI(TAG, "触摸屏 XPT2046 初始化完成");
}

bool hw_touch_read(uint16_t *x, uint16_t *y)
{
    uint8_t tx[3] = {0}, rx[3] = {0};
    spi_transaction_t t = { .length = 24, .tx_buffer = tx, .rx_buffer = rx };

    tx[0] = XPT2046_CMD_X_READ;
    spi_device_transmit(touch_spi, &t);
    *x = ((rx[1] << 8) | rx[2]) >> 3;

    tx[0] = XPT2046_CMD_Y_READ;
    spi_device_transmit(touch_spi, &t);
    *y = ((rx[1] << 8) | rx[2]) >> 3;

    return (*x < 4095 && *y < 4095);
}

/* ============================================================
 * LVGL 集成
 * ============================================================ */
void hw_set_lvgl_disp(lv_disp_drv_t *disp) { g_disp_drv = disp; }

/* ============================================================
 * 一站式初始化 & 状态
 * ============================================================ */
void hw_init_all(void)
{
    hw_gpio_init();
    hw_spi_init();
    hw_buzzer_init();
    hw_backlight_init();
    hw_backlight_set(0);
    hw_display_init();
    hw_touch_init();
}

ecg_hw_state_t* hw_get_state(void) { return &hw_state; }


static uint16_t s_test_buf[LV_BUFFER_SIZE]; // 480x25=12000 pixels

void hw_test_display_pattern(void) {
    if (lcd_panel == NULL) {
        ESP_LOGE(TAG, "lcd_panel is NULL");
        return;
    }
    
    // 填充测试图案：交替行红色/蓝色
    for (int y = 0; y < 25; y++) {
        for (int x = 0; x < 480; x++) {
            if (y % 2 == 0) {
                s_test_buf[y*480 + x] = 0xF800; // Red
            } else {
                s_test_buf[y*480 + x] = 0x001F; // Blue
            }
        }
    }
    
    ESP_LOGI(TAG, "Sending test pattern to display...");
    esp_lcd_panel_draw_bitmap(lcd_panel, 0, 0, 480, 25, s_test_buf);
    ESP_LOGI(TAG, "Test pattern sent");
}
