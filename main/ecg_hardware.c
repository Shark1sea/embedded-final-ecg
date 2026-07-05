
#include "ecg_hardware.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ili9488.h"
#include "lvgl.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"

static const char *TAG = "ECG_HW";

// 内部状态
static ecg_hw_state_t hw_state = {
    .is_sampling = false,
    .is_transmitting = false
};

static adc_continuous_handle_t adc_handle = NULL;
static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
static esp_lcd_panel_handle_t lcd_panel_handle = NULL;
static spi_device_handle_t touch_spi_handle = NULL;

// LVGL display 句柄，由 hw_set_lvgl_display 设置，在 notify_lvgl_flush_ready 中用于调用 lv_display_flush_ready
static lv_display_t *g_lvgl_disp = NULL;

// 通知 LVGL 刷新完成的回调
static bool IRAM_ATTR notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    if (g_lvgl_disp != NULL) {
        lv_display_flush_ready(g_lvgl_disp);
    }
    return false;
}

void hw_gpio_init(void) {
    ESP_LOGI(TAG, "初始化GPIO");
    
    // ADC状态引脚配置为输入
    gpio_config_t adc_status_config = {
        .pin_bit_mask = (1ULL << ADC_LON_GPIO_NUM) | (1ULL << ADC_LOP_GPIO_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&adc_status_config);
    
    // ADC控制引脚配置为输出
    gpio_config_t adc_ctrl_config = {
        .pin_bit_mask = (1ULL << ADC_CONTROL_GPIO_NUM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&adc_ctrl_config);
    gpio_set_level(ADC_CONTROL_GPIO_NUM, 0);
    
    // 蜂鸣器引脚配置
    gpio_config_t buzzer_config = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO_NUM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&buzzer_config);
    
    ESP_LOGI(TAG, "GPIO初始化完成");
}

void hw_spi_init(void) {
    ESP_LOGI(TAG, "初始化SPI总线");
    
    // 显示屏SPI总线
    spi_bus_config_t display_bus_config = {
        .mosi_io_num = DISPLAY_SPI_MOSI,
        .miso_io_num = DISPLAY_SPI_MISO,
        .sclk_io_num = DISPLAY_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = DISPLAY_H_RES * 10 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &display_bus_config, SPI_DMA_CH_AUTO));
    
    // 触摸屏SPI总线
    spi_bus_config_t touch_bus_config = {
        .mosi_io_num = TOUCH_SPI_MOSI,
        .miso_io_num = TOUCH_SPI_MISO,
        .sclk_io_num = TOUCH_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &touch_bus_config, SPI_DMA_CH_AUTO));
    
    ESP_LOGI(TAG, "SPI总线初始化完成");
}

void hw_adc_init(void) {
    ESP_LOGI(TAG, "初始化ADC连续模式");
    
    adc_continuous_handle_cfg_t adc_cfg = {
        .max_store_buf_size = 8192,
        .conv_frame_size = ADC_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_cfg, &adc_handle));
    
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    
    adc_channel_t channel = ADC_CHANNEL_0;
    adc_digi_pattern_config_t pattern_cfg = {
        .atten = ADC_ATTEN,
        .channel = channel,
        .unit = ADC_UNIT,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    dig_cfg.adc_pattern = &pattern_cfg;
    dig_cfg.pattern_num = 1;
    
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    
    ESP_LOGI(TAG, "ADC初始化完成");
}

void hw_buzzer_init(void) {
    ESP_LOGI(TAG, "初始化蜂鸣器");
    
    ledc_timer_config_t timer_cfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .timer_num = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_LEDC_DUTY_RES,
        .freq_hz = BUZZER_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    
    ledc_channel_config_t channel_cfg = {
        .gpio_num = BUZZER_GPIO_NUM,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    
    ESP_LOGI(TAG, "蜂鸣器初始化完成");
}

void hw_display_init(void) {
    ESP_LOGI(TAG, "初始化显示屏");
    
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = DISPLAY_SPI_CS,
        .dc_gpio_num = DISPLAY_DC,
        .spi_mode = 0,
        .pclk_hz = DISPLAY_REFRESH_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0
        }
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &lcd_io_handle));
    
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = DISPLAY_RESET,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 18,
        .flags = {
            .reset_active_high = 0
        },
        .vendor_config = NULL
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(lcd_io_handle, &panel_config, LV_BUFFER_SIZE, &lcd_panel_handle));
    
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_panel_handle, true));       // 交换X和Y轴
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel_handle, true, false)); // 该语句将显示内容镜像
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel_handle, true));
    
    ESP_LOGI(TAG, "显示屏初始化完成");
}

void hw_touch_init(void) {
    ESP_LOGI(TAG, "初始化触摸屏");
    
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = TOUCH_SPI_CS,
        .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev_cfg, &touch_spi_handle));
    
    ESP_LOGI(TAG, "触摸屏初始化完成");
}

void hw_backlight_init(void) {
    ESP_LOGI(TAG, "初始化背光PWM");
    
    ledc_timer_config_t timer_cfg = {
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .timer_num = BACKLIGHT_LEDC_TIMER,
        .duty_resolution = BACKLIGHT_LEDC_TIMER_RES,
        .freq_hz = BACKLIGHT_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    
    ledc_channel_config_t channel_cfg = {
        .gpio_num = DISPLAY_BACKLIGHT,
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .channel = BACKLIGHT_LEDC_CHANNEL,
        .timer_sel = BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 1, // 背光可能需要反向
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    
    ESP_LOGI(TAG, "背光PWM初始化完成");
}

void hw_buzzer_on(uint32_t freq_hz) {
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, freq_hz);
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, (1 << LEDC_TIMER_10_BIT) / 2);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

void hw_buzzer_off(void) {
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

void hw_backlight_set(uint8_t brightness) {
    if (brightness > BACKLIGHT_MAX) brightness = BACKLIGHT_MAX;
    if (brightness < BACKLIGHT_MIN) brightness = BACKLIGHT_MIN;
    
    // 反向，因为设置了 output_invert
    uint32_t duty = 1023 - (1023 * brightness) / 100;
    ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL, duty);
    ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL);
}

void hw_adc_start(void) {
    if (hw_state.is_sampling) {
        ESP_LOGW(TAG, "ADC已在运行");
        return;
    }
    
    ESP_LOGI(TAG, "启动ADC采样");
    gpio_set_level(ADC_CONTROL_GPIO_NUM, 1);
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    hw_state.is_sampling = true;
}

void hw_adc_stop(void) {
    if (!hw_state.is_sampling) {
        ESP_LOGW(TAG, "ADC未在运行");
        return;
    }
    
    ESP_LOGI(TAG, "停止ADC采样");
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    gpio_set_level(ADC_CONTROL_GPIO_NUM, 0);
    hw_state.is_sampling = false;
}

void hw_read_adc_status(bool *lon_ok, bool *lop_ok) {
    if (lon_ok != NULL) {
        *lon_ok = !gpio_get_level(ADC_LON_GPIO_NUM);
    }
    if (lop_ok != NULL) {
        *lop_ok = !gpio_get_level(ADC_LOP_GPIO_NUM);
    }
}

bool hw_adc_read(uint8_t *buffer, uint32_t buf_size, uint32_t *out_size) {
    esp_err_t ret = adc_continuous_read(adc_handle, buffer, buf_size, out_size, 0);
    return (ret == ESP_OK);
}

ecg_hw_state_t* hw_get_state(void) {
    return &hw_state;
}

void* hw_get_lcd_panel_handle(void) {
    return (void*)lcd_panel_handle;
}

void hw_set_lvgl_display(lv_display_t *disp) {
    g_lvgl_disp = disp;
}

static uint16_t s_test_buf[LV_BUFFER_SIZE]; // 480x25=12000 pixels

void hw_test_display_pattern(void) {
    if (lcd_panel_handle == NULL) {
        ESP_LOGE(TAG, "lcd_panel_handle is NULL");
        return;
    }
    
    // 填充测试图案：交替行红色/蓝色
    for (int y = 0; y < 25; y++) {
        for (int x = 0; x < 480; x++) {
            if (y % 2 == 0) {
                // RGB565: Red
                s_test_buf[y*480 + x] = 0xF800;
            } else {
                // RGB565: Blue
                s_test_buf[y*480 + x] = 0x001F;
            }
        }
    }
    
    ESP_LOGI(TAG, "Sending test pattern to display...");
    // 直接发送给硬件 (x_end, y_end 是 exclusive)
    esp_lcd_panel_draw_bitmap(lcd_panel_handle, 0, 0, 480, 25, s_test_buf);
    ESP_LOGI(TAG, "Test pattern sent");
}
