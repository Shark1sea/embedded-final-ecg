| Supported Targets | ESP32 | ESP32-S3 |
| ----------------- | ----- | -------- |

# 智能心电监测系统 (ECG)

基于 ESP-IDF 的嵌入式心电信号采集与智能诊断系统，支持实时波形显示、心率监测、心律失常检测与 AI 辅助诊断。

## 功能特性

- **心电信号采集**: ADC 连续采样 (1000 Hz)，FIR 低通滤波 (40 Hz) + 4 倍降采样至 250 Hz
- **QRS 波检测**: Pan-Tompkins 实时算法，精准检测心室除极波
- **心率监测**: 逐拍心率计算，RR 间期统计
- **心律失常检测** (规则引擎):
  - 心动过速 / 心动过缓
  - 心律不齐 (RR 间期变异分析)
  - 疑似房颤 (RR 间期绝对不齐)
  - 疑似早搏 (短 RR 间期)
- **AI 诊断**: 分层诊断架构，支持 TFLite Micro 模型推理 (已预留接口)
- **LVGL 图形界面**: ILI9488 480×320 TFT + XPT2046 触摸，实时波形绘制
- **跨核安全设计**: FreeRTOS 双核 (Core 0 UI / Core 1 采样+DSP)，portMUX spinlock 保护

## 硬件连接

| 外设 | 引脚 (ESP32-S3) | 引脚 (ESP32) |
|------|-----------------|--------------|
| ADC 输入 | GPIO 1 | GPIO 34 |
| ILI9488 SPI CLK | GPIO 14 | GPIO 14 |
| ILI9488 SPI MOSI | GPIO 15 | GPIO 15 |
| ILI9488 SPI MISO | GPIO 2 | GPIO 2 |
| ILI9488 SPI CS | GPIO 16 | GPIO 16 |
| ILI9488 DC | GPIO 17 | GPIO 17 |
| ILI9488 RST | GPIO 4 | GPIO 4 |
| 背光 PWM | GPIO 18 | GPIO 18 |
| XPT2046 SPI CLK | GPIO 12 | GPIO 12 |
| XPT2046 SPI MOSI | GPIO 20 | GPIO 20 |
| XPT2046 SPI MISO | GPIO 19 | GPIO 19 |
| XPT2046 SPI CS | GPIO 21 | GPIO 21 |

## 软件依赖

| 组件 | 版本 | 用途 |
|------|------|------|
| ESP-IDF | ≥6.0.1 | 基础框架 |
| LVGL | <9.0.0 | 图形界面 |
| esp-dsp | ^1.6.4 | FIR 滤波 (dsps_fird_s16) |
| esp_lcd_ili9488 | ^1.1.1 | LCD 驱动 |

## 项目结构

```
├── CMakeLists.txt              # 顶层 CMake
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                  # 主入口，任务调度，UI 回调
│   ├── ecg_config.h            # 引脚、采样参数、阈值配置
│   ├── ecg_hardware.c/h        # 硬件抽象 (ADC, SPI, 背光, 触摸)
│   ├── ecg_dsp.c/h             # DSP: FIR 滤波、降采样、QRS 检测
│   ├── ecg_ai_diagnosis.c/h    # AI 诊断: 规则引擎 + NN 预留接口
│   ├── ecg_ui.c/h              # LVGL UI: 界面创建、波形绘制、跨核同步
│   ├── panTompkins/            # Pan-Tompkins QRS 检测算法
│   └── idf_component.yml       # 组件依赖声明
├── managed_components/         # ESP-IDF 组件管理器下载的依赖
└── sdkconfig                   # 项目配置
```

## 编译与烧录

```bash
# 设置目标芯片
idf.py set-target esp32s3

# 编译
idf.py build

# 烧录并查看串口输出
idf.py -p /dev/ttyUSB0 flash monitor
```

## 使用说明

1. 上电后进入主界面，显示波形区域、心率标签、警告/诊断标签
2. 点击 **开始采集** 启动 ECG 采样
3. 波形区域实时显示滤波后的心电信号
4. 心率标签每 5 次 QRS 检测更新一次
5. 警告标签自动显示：心率异常提示 / AI 诊断结果 (每 5s 更新)
6. 滑动界面进入 **设置页**：调节背光亮度、心率报警阈值
7. 点击 **停止采集** 结束采样

## 信号处理链路

```
ADC (1000Hz) → FIR 低通 (64-tap, Fc=40Hz) → 降采样 ×4
    → Pan-Tompkins QRS 检测 → RR 间期 → 心率
    → 规则引擎 → 心律失常分类
    → AI 诊断 (每 5s) → UI 标签
```

## 滤波器设计

FIR 低通滤波器参数：`Fs=1000Hz, Fc=40Hz, 64-tap Kaiser (β=6)`，由 MATLAB `fir1` 设计，Q15 定点量化后部署到 esp-dsp。

## License

MIT

