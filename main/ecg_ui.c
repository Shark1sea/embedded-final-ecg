/**
 * @file    ecg_ui.c
 * @brief   ECG UI 模块: LVGL 初始化、界面创建、事件回调、数据更新
 * @note    从 main.c 中解耦，所有 UI 逻辑集中于此
 */

#include "ecg_ui.h"
#include "ecg_hardware.h"
#include "ecg_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/portmacro.h"

/* ---- 外部字体 ---- */
LV_FONT_DECLARE(JiDianHei_18)

static const char *TAG = "UI";

/* ============================================================
 * LVGL 内部变量
 * ============================================================ */
static lv_disp_drv_t  disp_drv;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t    *buf1, *buf2;
static lv_indev_drv_t indev_drv;
static esp_timer_handle_t tick_timer;

/* ---- 主屏幕布局 ---- */
static lv_obj_t *scr;
static int16_t col_dsc[] = { 80, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
static int16_t row_dsc[] = { LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };

/* ---- 菜单栏 ---- */
static lv_obj_t *menu_cont, *menu_btn_cont, *indi_cont, *indicator;
static lv_obj_t *menu_btn_1, *menu_btn_1_label;
static lv_obj_t *menu_btn_2, *menu_btn_2_label;
static lv_obj_t *cont, *page_1, *page_2;
static int16_t menu_col[] = { 76, 4, LV_GRID_TEMPLATE_LAST };
static int16_t menu_row[] = { LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
static const lv_coord_t menu_btn_h = 80;

/* ---- Page 1: ECG 采集页 ---- */
static lv_obj_t *ecg_btn, *ecg_btn_label;
static lv_obj_t *transmit_btn, *transmit_btn_label;
static lv_obj_t *freq_label, *warn_label, *hr_label;
static lv_obj_t *ecg_cont, *ecg_line, *ecg_indicator;
static lv_point_t ecg_pts[ECG_POINT_COUNT];
static uint16_t   ecg_pt_idx;
static portMUX_TYPE ecg_pts_lock = portMUX_INITIALIZER_UNLOCKED;  // 跨核 spinlock
static int16_t col_p1[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
static int16_t row_p1[] = { 60, 60, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };

/* ---- Page 2: 设置页 ---- */
static lv_obj_t *set_head_cont, *set_head_label, *reset_btn, *reset_btn_label;
static lv_obj_t *set_cont, *set_list;
static lv_obj_t *brightness_item, *brightness_label, *brightness_bar, *brightness_val_label;
static lv_obj_t *high_alarm_item, *high_alarm_label, *high_alarm_bar, *high_alarm_val_label;
static lv_obj_t *low_alarm_item, *low_alarm_label, *low_alarm_bar, *low_alarm_val_label;
static int16_t col_p2[] = { LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
static int16_t row_p2[] = { 60, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };

/* ---- 样式 ---- */
static lv_style_t style_screen, style_menu_btn, style_usual_label;
static lv_style_t style_warn, style_warn_ok, style_hr;
static lv_style_t style_ecg_start, style_ecg_stop;
static lv_style_t style_tx_start, style_tx_stop;
static lv_style_t style_ecg_line, style_ecg_ind;

/* ---- 回调指针 ---- */
static ui_ecg_start_cb_t      cb_ecg_start;
static ui_ecg_stop_cb_t       cb_ecg_stop;
static ui_transmit_start_cb_t cb_tx_start;
static ui_transmit_stop_cb_t  cb_tx_stop;
static ui_backlight_set_cb_t  cb_bl_set;
static ui_alarm_high_set_cb_t cb_alarm_high;
static ui_alarm_low_set_cb_t  cb_alarm_low;

/* ---- 内部状态 ---- */
static bool ecg_active;
static bool tx_active;
static uint8_t  bl_value   = BACKLIGHT_DEFAULT;
static uint16_t alarm_high = HIGH_ALARM_DEFAULT;
static uint16_t alarm_low  = LOW_ALARM_DEFAULT;

/* ---- 跨核共享缓冲区 (Core 1 写入, Core 0 读取并刷新 LVGL) ---- */
static volatile float    pending_hr = -1.0f;      // <0 无更新, >=0 有心率
static volatile bool     hr_dirty;
static volatile bool     waveform_dirty;
static volatile bool     indicator_dirty;
static volatile bool     need_indicator_show;
static volatile bool     reset_dirty;
static volatile bool     warning_dirty;
static volatile bool     pending_warn;              // volatile: 跨核读写
static volatile char     pending_text[48];          // volatile: 跨核读写, 防止字符串撕裂
static portMUX_TYPE      warning_lock = portMUX_INITIALIZER_UNLOCKED;  // 保护 warning 数据

/* ============================================================
 * 内部辅助: 动画回调
 * ============================================================ */
static void anim_set_y_cb(void *obj, int32_t y) {
    lv_obj_set_y((lv_obj_t *)obj, (lv_coord_t)y);
}

/* ============================================================
 * LVGL 刷新回调
 * ============================================================ */
static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *map) {
    esp_lcd_panel_handle_t panel = hw_get_lcd_panel();
    esp_lcd_panel_draw_bitmap(panel,
        area->x1, area->y1, area->x2 + 1, area->y2 + 1, map);
    lv_disp_flush_ready(drv);
}

/* ============================================================
 * LVGL Tick 定时器
 * ============================================================ */
static void IRAM_ATTR tick_cb(void *param) {
    lv_tick_inc(LVGL_UPDATE_PERIOD_MS);
}

/* ============================================================
 * 触摸读取
 * ============================================================ */
static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    static lv_point_t last;
    uint16_t x, y;
    bool touched = hw_touch_read(&x, &y);
    if (touched) {
        last.x = x * DISPLAY_H_RES / 4096;
        last.y = DISPLAY_V_RES - y * DISPLAY_V_RES / 4096;
        last.x = LV_CLAMP(0, last.x, DISPLAY_H_RES - 1);
        last.y = LV_CLAMP(0, last.y, DISPLAY_V_RES - 1);
    }
    data->point = last;
    data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

/* ============================================================
 * 样式初始化
 * ============================================================ */
static void styles_init(void) {
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, lv_color_black());
    lv_style_set_border_width(&style_screen, 0);

    lv_style_init(&style_menu_btn);
    lv_style_set_bg_color(&style_menu_btn, lv_color_white());
    lv_style_set_text_color(&style_menu_btn, lv_color_black());
    lv_style_set_text_font(&style_menu_btn, &JiDianHei_18);
    lv_style_set_radius(&style_menu_btn, 0);
    lv_style_set_border_width(&style_menu_btn, 0);
    lv_style_set_shadow_width(&style_menu_btn, 0);
    lv_style_set_pad_all(&style_menu_btn, 0);

    lv_style_init(&style_usual_label);
    lv_style_set_text_color(&style_usual_label, lv_color_black());
    lv_style_set_text_font(&style_usual_label, &JiDianHei_18);
    lv_style_set_text_align(&style_usual_label, LV_TEXT_ALIGN_CENTER);

    lv_style_init(&style_warn);
    lv_style_set_text_color(&style_warn, lv_color_hex(0xFF0000));
    lv_style_set_text_font(&style_warn, &JiDianHei_18);
    lv_style_set_text_align(&style_warn, LV_TEXT_ALIGN_CENTER);

    lv_style_init(&style_warn_ok);
    lv_style_set_text_color(&style_warn_ok, lv_color_hex(0x00EE00));
    lv_style_set_text_font(&style_warn_ok, &JiDianHei_18);
    lv_style_set_text_align(&style_warn_ok, LV_TEXT_ALIGN_CENTER);

    lv_style_init(&style_hr);
    lv_style_set_text_color(&style_hr, lv_color_black());
    lv_style_set_text_font(&style_hr, &JiDianHei_18);
    lv_style_set_text_align(&style_hr, LV_TEXT_ALIGN_CENTER);

    lv_style_init(&style_ecg_start);
    lv_style_set_bg_color(&style_ecg_start, lv_color_hex(0x00EE00));
    lv_style_set_text_color(&style_ecg_start, lv_color_white());
    lv_style_set_shadow_width(&style_ecg_start, 0);

    lv_style_init(&style_ecg_stop);
    lv_style_set_bg_color(&style_ecg_stop, lv_color_hex(0xFF4040));
    lv_style_set_text_color(&style_ecg_stop, lv_color_white());
    lv_style_set_shadow_width(&style_ecg_stop, 0);

    lv_style_init(&style_tx_start);
    lv_style_set_bg_color(&style_tx_start, lv_color_hex(0x0080FF));
    lv_style_set_text_color(&style_tx_start, lv_color_white());
    lv_style_set_shadow_width(&style_tx_start, 0);

    lv_style_init(&style_tx_stop);
    lv_style_set_bg_color(&style_tx_stop, lv_color_hex(0xFF4040));
    lv_style_set_text_color(&style_tx_stop, lv_color_white());
    lv_style_set_shadow_width(&style_tx_stop, 0);

    lv_style_init(&style_ecg_line);
    lv_style_set_line_color(&style_ecg_line, lv_color_hex(0xF00000));
    lv_style_set_line_width(&style_ecg_line, 2);
    lv_style_set_shadow_width(&style_ecg_line, 0);

    lv_style_init(&style_ecg_ind);
    lv_style_set_bg_color(&style_ecg_ind, lv_color_hex(0x0080FF));
    lv_style_set_border_width(&style_ecg_ind, 0);
    lv_style_set_width(&style_ecg_ind, 2);
    lv_style_set_height(&style_ecg_ind, ECG_DISPLAY_HEIGHT);
}

/* ============================================================
 * 菜单按钮事件
 * ============================================================ */
static void menu_btn_event_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    lv_coord_t btn_y = lv_obj_get_y(btn);
    lv_coord_t target_y = btn_y + (menu_btn_h - 80) / 2;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, indicator);
    lv_anim_set_exec_cb(&a, anim_set_y_cb);
    lv_anim_set_values(&a, lv_obj_get_y(indicator), target_y);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_set_time(&a, 300);
    lv_anim_start(&a);

    if (btn == menu_btn_1) {
        lv_obj_clear_flag(page_1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_2, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(page_2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_1, LV_OBJ_FLAG_HIDDEN);
    }
}

/* ============================================================
 * ECG 按钮事件
 * ============================================================ */
static void ecg_btn_event_cb(lv_event_t *e) {
    bool lon, lop;
    hw_read_adc_status(&lon, &lop);

    if (ecg_active) {
        // 停止采集
        if (cb_ecg_stop) cb_ecg_stop();
        ecg_active = false;
        lv_obj_add_style(ecg_btn, &style_ecg_start, 0);
        lv_label_set_text(ecg_btn_label, "开始采集");
        ui_update_warning("停止采集...", false);
    } else {
        // 检查导联
        if (!lon && !lop) {
            ui_update_warning("请先连接导联", true);
            return;
        }
        if (lon != lop) {
            ui_update_warning("导联连接不正确", true);
            return;
        }
        // 开始采集
        ecg_active = true;
        lv_obj_add_style(ecg_btn, &style_ecg_stop, 0);
        lv_label_set_text(ecg_btn_label, "停止采集");
        ui_update_warning("启动采集...", false);
        ui_update_heart_rate(-1);
        ui_reset_waveform();
        if (cb_ecg_start) cb_ecg_start();
    }
}

/* ============================================================
 * 传输按钮事件
 * ============================================================ */
static void transmit_btn_event_cb(lv_event_t *e) {
    if (tx_active) {
        tx_active = false;
        lv_obj_add_style(transmit_btn, &style_tx_start, 0);
        lv_label_set_text(transmit_btn_label, "串口传输");
        if (cb_tx_stop) cb_tx_stop();
    } else {
        if (!ecg_active) {
            ui_update_warning("请先开始采集", true);
            return;
        }
        tx_active = true;
        lv_obj_add_style(transmit_btn, &style_tx_stop, 0);
        lv_label_set_text(transmit_btn_label, "停止传输");
        if (cb_tx_start) cb_tx_start();
    }
}

/* ============================================================
 * 设置事件
 * ============================================================ */
static void reset_btn_event_cb(lv_event_t *e) {
    lv_slider_set_value(brightness_bar, BACKLIGHT_DEFAULT, LV_ANIM_OFF);
    lv_label_set_text_fmt(brightness_val_label, "%d%%", BACKLIGHT_DEFAULT);
    lv_slider_set_value(high_alarm_bar, HIGH_ALARM_DEFAULT, LV_ANIM_OFF);
    lv_label_set_text_fmt(high_alarm_val_label, "%d bpm", HIGH_ALARM_DEFAULT);
    lv_slider_set_value(low_alarm_bar, LOW_ALARM_DEFAULT, LV_ANIM_OFF);
    lv_label_set_text_fmt(low_alarm_val_label, "%d bpm", LOW_ALARM_DEFAULT);
}

static void brightness_event_cb(lv_event_t *e) {
    int v = lv_slider_get_value(brightness_bar);
    if (v >= BACKLIGHT_MIN && v <= BACKLIGHT_MAX) {
        bl_value = v;
        lv_label_set_text_fmt(brightness_val_label, "%d%%", v);
        if (cb_bl_set) cb_bl_set(v);
    }
}

static void high_alarm_event_cb(lv_event_t *e) {
    int v = lv_slider_get_value(high_alarm_bar);
    if (v >= HIGH_ALARM_MIN && v <= HIGH_ALARM_MAX) {
        alarm_high = v;
        lv_label_set_text_fmt(high_alarm_val_label, "%d bpm", v);
        if (cb_alarm_high) cb_alarm_high(v);
    }
}

static void low_alarm_event_cb(lv_event_t *e) {
    int v = lv_slider_get_value(low_alarm_bar);
    if (v >= LOW_ALARM_MIN) {
        alarm_low = v;
        lv_label_set_text_fmt(low_alarm_val_label, "%d bpm", v);
        if (cb_alarm_low) cb_alarm_low(v);
    }
}

/* ============================================================
 * 界面创建
 * ============================================================ */
static void create_menu(lv_obj_t *parent) {
    menu_cont = lv_obj_create(parent);
    lv_obj_set_size(menu_cont, 80, DISPLAY_V_RES);
    lv_obj_set_style_bg_color(menu_cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(menu_cont, 0, 0);
    lv_obj_set_style_pad_all(menu_cont, 0, 0);
    lv_obj_set_style_radius(menu_cont, 0, 0);
    lv_obj_clear_flag(menu_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_cell(menu_cont, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_CENTER, 0, 1);
    lv_obj_set_grid_dsc_array(menu_cont, menu_col, menu_row);
    lv_obj_set_style_pad_column(menu_cont, 0, 0);
    lv_obj_set_style_pad_row(menu_cont, 0, 0);

    // 按钮容器
    menu_btn_cont = lv_obj_create(menu_cont);
    lv_obj_set_size(menu_btn_cont, 76, DISPLAY_V_RES);
    lv_obj_set_style_bg_color(menu_btn_cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(menu_btn_cont, 0, 0);
    lv_obj_set_style_pad_all(menu_btn_cont, 0, 0);
    lv_obj_set_style_radius(menu_btn_cont, 0, 0);
    lv_obj_clear_flag(menu_btn_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(menu_btn_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(menu_btn_cont, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
    lv_obj_set_grid_cell(menu_btn_cont, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_CENTER, 0, 1);

    // 按钮1
    menu_btn_1 = lv_btn_create(menu_btn_cont);
    lv_obj_set_size(menu_btn_1, 76, menu_btn_h);
    lv_obj_add_style(menu_btn_1, &style_menu_btn, 0);
    lv_obj_add_event_cb(menu_btn_1, menu_btn_event_cb, LV_EVENT_CLICKED, NULL);
    menu_btn_1_label = lv_label_create(menu_btn_1);
    lv_label_set_text(menu_btn_1_label, "心\n电\n采\n集");
    lv_obj_center(menu_btn_1_label);
    lv_obj_add_style(menu_btn_1_label, &style_usual_label, 0);

    // 按钮2
    menu_btn_2 = lv_btn_create(menu_btn_cont);
    lv_obj_set_size(menu_btn_2, 76, menu_btn_h);
    lv_obj_add_style(menu_btn_2, &style_menu_btn, 0);
    lv_obj_add_event_cb(menu_btn_2, menu_btn_event_cb, LV_EVENT_CLICKED, NULL);
    menu_btn_2_label = lv_label_create(menu_btn_2);
    lv_label_set_text(menu_btn_2_label, "系\n统\n设\n置");
    lv_obj_center(menu_btn_2_label);
    lv_obj_add_style(menu_btn_2_label, &style_usual_label, 0);

    // 指示线容器
    indi_cont = lv_obj_create(menu_cont);
    lv_obj_set_size(indi_cont, 4, DISPLAY_V_RES);
    lv_obj_set_style_bg_color(indi_cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(indi_cont, 0, 0);
    lv_obj_set_style_pad_all(indi_cont, 0, 0);
    lv_obj_set_style_radius(indi_cont, 0, 0);
    lv_obj_clear_flag(indi_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_cell(indi_cont, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_CENTER, 0, 1);

    indicator = lv_obj_create(indi_cont);
    lv_obj_set_size(indicator, 4, 80);
    lv_obj_set_style_bg_color(indicator, lv_color_hex(0x0080FF), 0);
    lv_obj_set_style_border_width(indicator, 0, 0);
    lv_obj_set_style_radius(indicator, 2, 0);
}

static void create_page1(lv_obj_t *parent) {
    page_1 = lv_obj_create(parent);
    lv_obj_set_size(page_1, lv_pct(100), lv_pct(100));
    lv_obj_set_style_pad_all(page_1, 0, 0);
    lv_obj_set_style_radius(page_1, 0, 0);
    lv_obj_clear_flag(page_1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_dsc_array(page_1, col_p1, row_p1);
    lv_obj_set_style_pad_column(page_1, 0, 0);
    lv_obj_set_style_pad_row(page_1, 0, 0);
    lv_obj_set_style_pad_all(page_1, 0, 0);

    // ECG 按钮
    ecg_btn = lv_btn_create(page_1);
    lv_obj_set_size(ecg_btn, 110, 50);
    lv_obj_add_style(ecg_btn, &style_ecg_start, 0);
    lv_obj_add_event_cb(ecg_btn, ecg_btn_event_cb, LV_EVENT_CLICKED, NULL);
    ecg_btn_label = lv_label_create(ecg_btn);
    lv_label_set_text(ecg_btn_label, "开始采集");
    lv_obj_center(ecg_btn_label);
    lv_obj_add_style(ecg_btn_label, &style_usual_label, 0);
    lv_obj_set_grid_cell(ecg_btn, LV_GRID_ALIGN_CENTER, 0, 1,
                         LV_GRID_ALIGN_CENTER, 0, 1);

    // 传输按钮
    transmit_btn = lv_btn_create(page_1);
    lv_obj_set_size(transmit_btn, 110, 50);
    lv_obj_add_style(transmit_btn, &style_tx_start, 0);
    lv_obj_add_event_cb(transmit_btn, transmit_btn_event_cb, LV_EVENT_CLICKED, NULL);
    transmit_btn_label = lv_label_create(transmit_btn);
    lv_label_set_text(transmit_btn_label, "串口传输");
    lv_obj_center(transmit_btn_label);
    lv_obj_add_style(transmit_btn_label, &style_usual_label, 0);
    lv_obj_set_grid_cell(transmit_btn, LV_GRID_ALIGN_CENTER, 1, 1,
                         LV_GRID_ALIGN_CENTER, 0, 1);

    // 采样率标签
    freq_label = lv_label_create(page_1);
    lv_label_set_text_fmt(freq_label, "采样率: %d Hz",
                          ADC_SAMPLE_FREQ_HZ / DOWNSAMPLE_FACTOR);
    lv_obj_center(freq_label);
    lv_obj_add_style(freq_label, &style_usual_label, 0);
    lv_obj_set_grid_cell(freq_label, LV_GRID_ALIGN_CENTER, 2, 1,
                         LV_GRID_ALIGN_CENTER, 0, 1);

    // 心率标签
    hr_label = lv_label_create(page_1);
    lv_label_set_text(hr_label, "心率: -- bpm");
    lv_obj_add_style(hr_label, &style_hr, 0);
    lv_obj_set_grid_cell(hr_label, LV_GRID_ALIGN_CENTER, 0, 1,
                         LV_GRID_ALIGN_CENTER, 1, 1);

    // 警告标签
    warn_label = lv_label_create(page_1);
    lv_label_set_text(warn_label, "请先连接导联");
    lv_obj_add_style(warn_label, &style_warn_ok, 0);
    lv_obj_set_grid_cell(warn_label, LV_GRID_ALIGN_CENTER, 1, 2,
                         LV_GRID_ALIGN_CENTER, 1, 1);

    // ECG 波形容器
    ecg_cont = lv_obj_create(page_1);
    lv_obj_set_size(ecg_cont, ECG_POINT_COUNT, ECG_DISPLAY_HEIGHT);
    lv_obj_set_style_pad_all(ecg_cont, 0, 0);
    lv_obj_set_style_radius(ecg_cont, 0, 0);
    lv_obj_set_style_bg_color(ecg_cont, lv_color_hex(0xE0FFFF), 0);
    lv_obj_set_grid_cell(ecg_cont, LV_GRID_ALIGN_CENTER, 0, 3,
                         LV_GRID_ALIGN_CENTER, 2, 1);
    lv_obj_clear_flag(ecg_cont, LV_OBJ_FLAG_SCROLLABLE);

    // ECG 曲线
    ecg_line = lv_line_create(ecg_cont);
    lv_obj_add_style(ecg_line, &style_ecg_line, 0);
    lv_obj_align(ecg_line, LV_ALIGN_LEFT_MID, 0, 0);
    lv_line_set_y_invert(ecg_line, true);

    // 刷新指示器
    ecg_indicator = lv_obj_create(ecg_cont);
    lv_obj_add_style(ecg_indicator, &style_ecg_ind, 0);
    lv_obj_set_size(ecg_indicator, 2, ECG_DISPLAY_HEIGHT);
    lv_obj_set_pos(ecg_indicator, 0, 0);
    lv_obj_add_flag(ecg_indicator, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(ecg_line);
}

static void create_page2(lv_obj_t *parent) {
    page_2 = lv_obj_create(parent);
    lv_obj_set_size(page_2, lv_pct(100), lv_pct(100));
    lv_obj_set_style_pad_all(page_2, 0, 0);
    lv_obj_set_style_radius(page_2, 0, 0);
    lv_obj_clear_flag(page_2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(page_2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_grid_dsc_array(page_2, col_p2, row_p2);
    lv_obj_set_style_pad_column(page_2, 0, 0);
    lv_obj_set_style_pad_row(page_2, 0, 0);
    lv_obj_set_style_pad_all(page_2, 0, 0);

    // 头部
    set_head_cont = lv_obj_create(page_2);
    lv_obj_set_size(set_head_cont, lv_pct(100), 60);
    lv_obj_set_style_bg_color(set_head_cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(set_head_cont, 0, 0);
    lv_obj_set_style_pad_all(set_head_cont, 10, 0);
    lv_obj_set_style_radius(set_head_cont, 0, 0);
    lv_obj_clear_flag(set_head_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_cell(set_head_cont, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_CENTER, 0, 1);

    set_head_label = lv_label_create(set_head_cont);
    lv_label_set_text(set_head_label, "系统设置");
    lv_obj_set_align(set_head_label, LV_ALIGN_LEFT_MID);
    lv_obj_add_style(set_head_label, &style_usual_label, 0);

    reset_btn = lv_btn_create(set_head_cont);
    lv_obj_set_align(reset_btn, LV_ALIGN_RIGHT_MID);
    lv_obj_set_size(reset_btn, 80, 50);
    reset_btn_label = lv_label_create(reset_btn);
    lv_label_set_text(reset_btn_label, "重置");
    lv_obj_center(reset_btn_label);
    lv_obj_add_style(reset_btn, &style_usual_label, 0);
    lv_obj_add_event_cb(reset_btn, reset_btn_event_cb, LV_EVENT_CLICKED, NULL);

    // 设置列表
    set_cont = lv_obj_create(page_2);
    lv_obj_set_size(set_cont, lv_pct(100), 260);
    lv_obj_set_style_bg_color(set_cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(set_cont, 0, 0);
    lv_obj_set_style_pad_all(set_cont, 0, 0);
    lv_obj_set_style_radius(set_cont, 0, 0);
    lv_obj_set_grid_cell(set_cont, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_CENTER, 1, 1);

    set_list = lv_obj_create(set_cont);
    lv_obj_set_size(set_list, lv_pct(100), 260);
    lv_obj_set_style_pad_all(set_list, 10, 0);
    lv_obj_set_style_radius(set_list, 0, 0);
    lv_obj_set_style_bg_color(set_list, lv_color_white(), 0);
    lv_obj_set_style_border_width(set_list, 0, 0);
    lv_obj_set_flex_flow(set_list, LV_FLEX_FLOW_COLUMN);

    // 亮度项
    brightness_item = lv_obj_create(set_list);
    lv_obj_set_size(brightness_item, 380, 80);
    lv_obj_set_style_bg_color(brightness_item, lv_color_white(), 0);
    lv_obj_set_style_border_width(brightness_item, 2, 0);
    lv_obj_set_style_radius(brightness_item, 5, 0);
    lv_obj_set_flex_flow(brightness_item, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(brightness_item, LV_FLEX_ALIGN_SPACE_AROUND,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    brightness_label = lv_label_create(brightness_item);
    lv_label_set_text(brightness_label, "屏幕亮度");
    lv_obj_add_style(brightness_label, &style_usual_label, 0);

    brightness_bar = lv_slider_create(brightness_item);
    lv_obj_set_size(brightness_bar, 160, 20);
    lv_slider_set_range(brightness_bar, BACKLIGHT_MIN, BACKLIGHT_MAX);
    lv_slider_set_value(brightness_bar, BACKLIGHT_DEFAULT, LV_ANIM_OFF);
    lv_obj_add_event_cb(brightness_bar, brightness_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    brightness_val_label = lv_label_create(brightness_item);
    lv_label_set_text_fmt(brightness_val_label, "%d%%", BACKLIGHT_DEFAULT);
    lv_obj_add_style(brightness_val_label, &style_usual_label, 0);

    // 高心率项
    high_alarm_item = lv_obj_create(set_list);
    lv_obj_set_size(high_alarm_item, 380, 80);
    lv_obj_set_style_bg_color(high_alarm_item, lv_color_white(), 0);
    lv_obj_set_style_border_width(high_alarm_item, 2, 0);
    lv_obj_set_style_radius(high_alarm_item, 5, 0);
    lv_obj_set_flex_flow(high_alarm_item, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(high_alarm_item, LV_FLEX_ALIGN_SPACE_AROUND,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    high_alarm_label = lv_label_create(high_alarm_item);
    lv_label_set_text(high_alarm_label, "高心率阈值");
    lv_obj_add_style(high_alarm_label, &style_usual_label, 0);

    high_alarm_bar = lv_slider_create(high_alarm_item);
    lv_obj_set_size(high_alarm_bar, 160, 20);
    lv_slider_set_range(high_alarm_bar, HIGH_ALARM_MIN, HIGH_ALARM_MAX);
    lv_slider_set_value(high_alarm_bar, HIGH_ALARM_DEFAULT, LV_ANIM_OFF);
    lv_obj_add_event_cb(high_alarm_bar, high_alarm_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    high_alarm_val_label = lv_label_create(high_alarm_item);
    lv_label_set_text_fmt(high_alarm_val_label, "%d bpm", HIGH_ALARM_DEFAULT);
    lv_obj_add_style(high_alarm_val_label, &style_usual_label, 0);

    // 低心率项
    low_alarm_item = lv_obj_create(set_list);
    lv_obj_set_size(low_alarm_item, 380, 80);
    lv_obj_set_style_bg_color(low_alarm_item, lv_color_white(), 0);
    lv_obj_set_style_border_width(low_alarm_item, 2, 0);
    lv_obj_set_style_radius(low_alarm_item, 5, 0);
    lv_obj_set_flex_flow(low_alarm_item, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(low_alarm_item, LV_FLEX_ALIGN_SPACE_AROUND,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    low_alarm_label = lv_label_create(low_alarm_item);
    lv_label_set_text(low_alarm_label, "低心率阈值");
    lv_obj_add_style(low_alarm_label, &style_usual_label, 0);

    low_alarm_bar = lv_slider_create(low_alarm_item);
    lv_obj_set_size(low_alarm_bar, 160, 20);
    lv_slider_set_range(low_alarm_bar, LOW_ALARM_MIN, LOW_ALARM_MAX);
    lv_slider_set_value(low_alarm_bar, LOW_ALARM_DEFAULT, LV_ANIM_OFF);
    lv_obj_add_event_cb(low_alarm_bar, low_alarm_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    low_alarm_val_label = lv_label_create(low_alarm_item);
    lv_label_set_text_fmt(low_alarm_val_label, "%d bpm", LOW_ALARM_DEFAULT);
    lv_obj_add_style(low_alarm_val_label, &style_usual_label, 0);
}

/* ============================================================
 * 公共 API
 * ============================================================ */
void ui_init(void) {
    ESP_LOGI(TAG, "初始化 LVGL UI");

    lv_init();

    // 显示缓冲区
    buf1 = heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#if USE_DOUBLE_BUFFERING
    buf2 = heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#else
    buf2 = NULL;
#endif
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LV_BUFFER_SIZE);

    // 显示驱动
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = DISPLAY_H_RES;
    disp_drv.ver_res  = DISPLAY_V_RES;
    disp_drv.flush_cb = flush_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = hw_get_lcd_panel();
    lv_disp_drv_register(&disp_drv);
    hw_set_lvgl_disp(&disp_drv);

    // Tick 定时器
    esp_timer_create_args_t targs = { .callback = tick_cb, .name = "lvgl_tick" };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_UPDATE_PERIOD_MS * 1000));

    // 触摸
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;
    lv_indev_drv_register(&indev_drv);

    // 样式 & 界面
    styles_init();
    scr = lv_disp_get_scr_act(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_style(scr, &style_screen, 0);
    lv_obj_set_grid_dsc_array(scr, col_dsc, row_dsc);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_set_style_pad_column(scr, 0, 0);
    lv_obj_set_style_pad_row(scr, 0, 0);

    create_menu(scr);

    cont = lv_obj_create(scr);
    lv_obj_set_size(cont, DISPLAY_H_RES - 80, DISPLAY_V_RES);
    lv_obj_set_style_bg_color(cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_set_style_radius(cont, 0, 0);
    lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_CENTER, 0, 1);

    create_page1(cont);
    create_page2(cont);

    // 初始化波形
    memset(ecg_pts, 0, sizeof(ecg_pts));
    ecg_pt_idx = 0;

    ESP_LOGI(TAG, "LVGL UI 初始化完成");
}

uint32_t ui_tick(void) {
    uint32_t ret = lv_timer_handler();
    ui_flush();
    return ret;
}

/* ---- 将共享缓冲区的数据刷新到 LVGL (仅 Core 0 调用) ---- */
void ui_flush(void) {
    // 心率更新
    if (hr_dirty) {
        hr_dirty = false;
        float bpm = pending_hr;
        if (hr_label) {
            if (bpm < 0) {
                lv_label_set_text(hr_label, "心率: 计算中...");
            } else if (bpm < 1.0f) {
                lv_label_set_text(hr_label, "心率: -- bpm");
            } else {
                lv_label_set_text_fmt(hr_label, "心率: %d bpm", (int)(bpm + 0.5f));
            }
        }
    }

    // 波形更新 (跨核锁定保护 ecg_pts / ecg_pt_idx)
    if (waveform_dirty) {
        waveform_dirty = false;
        portENTER_CRITICAL(&ecg_pts_lock);
        uint16_t idx_snap = ecg_pt_idx;
        uint16_t valid = idx_snap < ECG_POINT_COUNT ? idx_snap : ECG_POINT_COUNT;
        // 复制一份快照避免 LVGL 在渲染时读取到正在被 Core1 修改的数据
        static lv_point_t pts_snap[ECG_POINT_COUNT];
        if (valid > 0) {
            memcpy(pts_snap, ecg_pts, valid * sizeof(lv_point_t));
        }
        portEXIT_CRITICAL(&ecg_pts_lock);
        if (ecg_line && valid > 0) {
            lv_line_set_points(ecg_line, pts_snap, valid);
        }
        if (ecg_indicator) {
            lv_obj_set_x(ecg_indicator, idx_snap);
        }
    }

    // 指示器显隐
    if (indicator_dirty) {
        indicator_dirty = false;
        if (ecg_indicator) {
            if (need_indicator_show) {
                lv_obj_clear_flag(ecg_indicator, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(ecg_indicator, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    // 波形重置
    if (reset_dirty) {
        reset_dirty = false;
        portENTER_CRITICAL(&ecg_pts_lock);
        memset(ecg_pts, 0, sizeof(ecg_pts));
        ecg_pt_idx = 0;
        portEXIT_CRITICAL(&ecg_pts_lock);
        if (ecg_line) {
            static const lv_point_t empty_pt = {0, 0};
            lv_line_set_points(ecg_line, &empty_pt, 0);
        }
        if (ecg_indicator) lv_obj_set_x(ecg_indicator, 0);
    }

    // 警告更新 (跨核锁定保护 pending_text / pending_warn)
    if (warning_dirty) {
        warning_dirty = false;
        portENTER_CRITICAL(&warning_lock);
        bool warn_snap = pending_warn;
        char text_snap[48];
        strncpy(text_snap, (const char *)pending_text, sizeof(text_snap) - 1);
        text_snap[sizeof(text_snap) - 1] = '\0';
        portEXIT_CRITICAL(&warning_lock);
        if (warn_label) {
            lv_label_set_text(warn_label, text_snap);
            if (warn_snap) {
                lv_obj_add_style(warn_label, &style_warn, 0);
            } else {
                lv_obj_add_style(warn_label, &style_warn_ok, 0);
            }
        }
    }
}

void ui_update_heart_rate(float bpm) {
    pending_hr = bpm;
    hr_dirty = true;
}

void ui_update_waveform(const int16_t *data, uint16_t count) {
    if (!data || !count) return;

    portENTER_CRITICAL(&ecg_pts_lock);
    for (uint16_t i = 0; i < count; i++) {
        int32_t y = 1 + ((int32_t)(data[i] - 11000) * (ECG_DISPLAY_MAX_Y - 1) / (14500 - 11500)) - 30;
        if (y < ECG_DISPLAY_MIN_Y) y = ECG_DISPLAY_MIN_Y;
        if (y > ECG_DISPLAY_MAX_Y) y = ECG_DISPLAY_MAX_Y;

        ecg_pts[ecg_pt_idx].x = ecg_pt_idx;
        ecg_pts[ecg_pt_idx].y = (lv_coord_t)y;
        ecg_pt_idx++;
        if (ecg_pt_idx >= ECG_POINT_COUNT) ecg_pt_idx = 0;
    }
    portEXIT_CRITICAL(&ecg_pts_lock);
    waveform_dirty = true;
}

void ui_update_warning(const char *text, bool is_warn) {
    if (!text) return;
    portENTER_CRITICAL(&warning_lock);
    strncpy((char *)pending_text, text, sizeof(pending_text) - 1);
    ((char *)pending_text)[sizeof(pending_text) - 1] = '\0';
    pending_warn = is_warn;
    portEXIT_CRITICAL(&warning_lock);
    warning_dirty = true;
}

void ui_update_sampling_rate_label(void) {
    if (!freq_label) return;
    lv_label_set_text_fmt(freq_label, "采样率: %d Hz",
                          ADC_SAMPLE_FREQ_HZ / DOWNSAMPLE_FACTOR);
}

void ui_update_diagnosis(const ecg_diag_result_t *d) {
    if (!d || !d->is_valid) return;
    bool is_abnormal = (d->diagnosis != DIAG_NORMAL && d->diagnosis != DIAG_UNKNOWN);
    ui_update_warning(d->text, is_abnormal);
}

void ui_set_sampling_indicator(bool active) {
    need_indicator_show = active;
    indicator_dirty = true;
}

void ui_reset_waveform(void) {
    reset_dirty = true;
}

void ui_set_callbacks(
    ui_ecg_start_cb_t      es, ui_ecg_stop_cb_t      ep,
    ui_transmit_start_cb_t ts, ui_transmit_stop_cb_t  tp,
    ui_backlight_set_cb_t  bl, ui_alarm_high_set_cb_t ah,
    ui_alarm_low_set_cb_t  al)
{
    cb_ecg_start  = es;
    cb_ecg_stop   = ep;
    cb_tx_start   = ts;
    cb_tx_stop    = tp;
    cb_bl_set     = bl;
    cb_alarm_high = ah;
    cb_alarm_low  = al;
}

bool ui_is_ecg_active(void)      { return ecg_active; }
bool ui_is_transmit_active(void) { return tx_active; }
uint8_t  ui_get_backlight(void)   { return bl_value; }
uint16_t ui_get_alarm_high(void)  { return alarm_high; }
uint16_t ui_get_alarm_low(void)   { return alarm_low; }
