#include "panTompkins.h"
#include <stdio.h> // 用于调试打印，可移除

// --- 算法常量定义 ---
#define FS 360                               // 采样率 (Hz)
#define WINDOWSIZE (int)(FS * 0.15)          // 移动积分窗口大小 (150 ms)
#define BUFFSIZE (FS * 2)                    // 环形缓冲区大小 (2秒)
#define DELAY (FS + FS/2)                    // 输出延迟(1.5秒), 用于支持回溯搜索。应大于rrmiss的最大可能值。
#define RR_AVG_SIZE 8
// --- 静态状态变量 ---
// 缓冲区 (滤波链)
static dataType signal[BUFFSIZE], dcblock[BUFFSIZE], lowpass[BUFFSIZE], highpass[BUFFSIZE], derivative[BUFFSIZE], squared[BUFFSIZE], integral[BUFFSIZE];
static bool outputSignal[BUFFSIZE]; // 输出决策缓冲区，用于处理回溯

// RR间期和心率相关
static int rr1[RR_AVG_SIZE], rr2[RR_AVG_SIZE];
static int rravg1 = 0, rravg2 = 0, rrlow = 0, rrhigh = 0, rrmiss = 0;
static float current_bpm = 0.0f;

// 计数器和索引
static long unsigned int sample = 0; // 总样本计数器
static long unsigned int lastQRS = 0;
static dataType lastSlope = 0;

// 峰值和阈值
static dataType peak_i = 0, peak_f = 0;
static dataType threshold_i1 = 0, threshold_i2 = 0, threshold_f1 = 0, threshold_f2 = 0;
static dataType spk_i = 0, spk_f = 0, npk_i = 0, npk_f = 0;

// 状态标志
static bool regular = true;


// --- 内部辅助函数 ---
/**
 * @brief 更新RR间期、平均值和相关阈值。
 *        将重复的逻辑封装起来，供正常检测和回溯搜索调用。
 * @param new_rr 新计算出的RR间期。
 */
static void update_rr_intervals(long new_rr) {
    bool prevRegular = regular;
    
    // 更新 rr1 缓冲区和 rravg1 (最近8个RR间期的平均值)
    rravg1 = 0;
    for(int i = 0; i < RR_AVG_SIZE - 1; i++) {
        rr1[i] = rr1[i+1];
        rravg1 += rr1[i];
    }
    rr1[RR_AVG_SIZE - 1] = new_rr;
    rravg1 += rr1[RR_AVG_SIZE - 1];
    rravg1 /= RR_AVG_SIZE;
    
    // 如果新RR间期在正常范围内，则更新 rr2 缓冲区和相关阈值
    if (rrlow == 0 || (new_rr >= rrlow && new_rr <= rrhigh)) {
        rravg2 = 0;
        for(int i = 0; i < 7; i++) {
            rr2[i] = rr2[i+1];
            rravg2 += rr2[i];
        }
        rr2[7] = new_rr;
        rravg2 += rr2[7];
        rravg2 /= 8;
        
        rrlow = (int)(0.92 * rravg2);
        rrhigh = (int)(1.16 * rravg2);
        rrmiss = (int)(1.66 * rravg2);
    }

    // 判断心率是否规律
    if(rravg1 == rravg2) {
        regular = true;
    } else {
        regular = false;
        // 如果心率从规律变为不规律，降低阈值以便检测T波等
        if(prevRegular) {
            threshold_i1 /= 2;
            threshold_f1 /= 2;
        }
    }
    
    // 更新BPM
    if (rravg1 > 0) {
        current_bpm = (60.0f * FS) / rravg1;
    }
}


// --- 公共接口函数实现 ---
void panTompkins_init(void) {
    // 重置所有状态变量为初始值
    for (int i = 0; i < BUFFSIZE; i++) {
        signal[i] = dcblock[i] = lowpass[i] = highpass[i] = derivative[i] = squared[i] = integral[i] = 0;
        outputSignal[i] = false;
    }
    for (int i = 0; i < 8; i++) {
        rr1[i] = rr2[i] = 0;
    }
    rravg1 = rravg2 = rrlow = rrhigh = rrmiss = 0;
    sample = lastQRS = 0;
    lastSlope = 0;
    peak_i = peak_f = 0;
    threshold_i1 = threshold_i2 = threshold_f1 = threshold_f2 = 0;
    spk_i = spk_f = npk_i = npk_f = 0;
    regular = true;
    current_bpm = 0.0f;
}

void panTompkins_process(dataType new_sample) {
    bool qrs = false;
    dataType currentSlope = 0;

    // --- 缓冲区管理 ---
    int current_ptr = sample % BUFFSIZE;
    int prev1_ptr = (sample - 1 + BUFFSIZE) % BUFFSIZE;
    int prev2_ptr = (sample - 2 + BUFFSIZE) % BUFFSIZE;

    // --- 【修正后】的信号处理链条 ---
    signal[current_ptr] = new_sample;

    // 1. DC Blocker: y(n) = x(n) - x(n-1) + R * y(n-1)
    if (sample > 0) {
        dcblock[current_ptr] = signal[current_ptr] - signal[prev1_ptr] + 0.995 * dcblock[prev1_ptr];
    } else {
        dcblock[current_ptr] = 0;
    }
    
    // 2. Low-pass Filter: y(n) = 2y(n-1) - y(n-2) + (x(n) - 2x(n-6) + x(n-12))/32
    if (sample > 1) {
        lowpass[current_ptr] = 2 * lowpass[prev1_ptr] - lowpass[prev2_ptr]
                               + (dcblock[current_ptr] 
                                  - 2 * dcblock[(sample - 6 + BUFFSIZE) % BUFFSIZE] 
                                  + dcblock[(sample - 12 + BUFFSIZE) % BUFFSIZE]) / 32;
    } else if (sample == 1) {
         lowpass[current_ptr] = 2 * lowpass[prev1_ptr] + (dcblock[current_ptr] - 2*dcblock[0] + dcblock[0]) / 32;
    } else {
        lowpass[current_ptr] = dcblock[current_ptr] / 32;
    }

    // 3. High-pass Filter: y(n) = y(n-1) - x(n)/32 + x(n-16) - x(n-17) + x(n-32)/32
    if (sample > 0) {
        highpass[current_ptr] = highpass[prev1_ptr] - lowpass[current_ptr] / 32.0
                                + lowpass[(sample - 16 + BUFFSIZE) % BUFFSIZE]
                                - lowpass[(sample - 17 + BUFFSIZE) % BUFFSIZE]
                                + lowpass[(sample - 32 + BUFFSIZE) % BUFFSIZE] / 32.0;
    } else {
        highpass[current_ptr] = 0;
    }
    
    // 4. Derivative Filter: y(n) = (1/4T) * [ x(n) - x(n-2) ]
    derivative[current_ptr] = highpass[current_ptr] - highpass[prev2_ptr];

    // 5. Squaring
    squared[current_ptr] = derivative[current_ptr] * derivative[current_ptr];

    // 6. Moving Window Integration
    integral[current_ptr] = 0;
    for (int i = 0; i < WINDOWSIZE; i++) {
        integral[current_ptr] += squared[(sample - i + BUFFSIZE) % BUFFSIZE];
    }
    integral[current_ptr] /= WINDOWSIZE;
    
    // --- 峰值检测与决策逻辑 ---
    if ((integral[current_ptr] >= threshold_i1) && (highpass[current_ptr] >= threshold_f1)) {
        peak_i = integral[current_ptr];
        peak_f = highpass[current_ptr];
        
        if (sample > lastQRS + FS / 5) {
            if (sample <= lastQRS + (unsigned long)(0.36 * FS)) {
                currentSlope = 0;
                for (int j = 0; j < 10; j++) {
                    if (sample > j && squared[(sample - j + BUFFSIZE) % BUFFSIZE] > currentSlope) {
                        currentSlope = squared[(sample - j + BUFFSIZE) % BUFFSIZE];
                    }
                }
                if (currentSlope < lastSlope / 2) {
                    qrs = false;
                } else {
                    lastSlope = currentSlope;
                    qrs = true;
                }
            } else {
                currentSlope = 0;
                for (int j = 0; j < 10; j++) {
                    if (sample > j && squared[(sample - j + BUFFSIZE) % BUFFSIZE] > currentSlope) {
                        currentSlope = squared[(sample - j + BUFFSIZE) % BUFFSIZE];
                    }
                }
                lastSlope = currentSlope;
                qrs = true;
            }

            if (qrs) {
                spk_i = 0.125 * peak_i + 0.875 * spk_i;
                spk_f = 0.125 * peak_f + 0.875 * spk_f;
            } else {
                npk_i = 0.125 * peak_i + 0.875 * npk_i;
                npk_f = 0.125 * peak_f + 0.875 * npk_f;
            }
        } else {
            npk_i = 0.125 * peak_i + 0.875 * npk_i;
            npk_f = 0.125 * peak_f + 0.875 * npk_f;
            qrs = false;
        }
    }

    // --- R峰确认与回溯搜索 ---
    if (qrs) {
        update_rr_intervals(sample - lastQRS);
        lastQRS = sample;
    } else {
        if (rrmiss > 0 && sample - lastQRS > (unsigned long)rrmiss) {
            long unsigned int search_start = lastQRS + FS / 5;
            for (long unsigned int i = search_start; i < sample; i++) {
                int search_ptr = i % BUFFSIZE;
                if ((integral[search_ptr] > threshold_i2) && (highpass[search_ptr] > threshold_f2)) {
                    currentSlope = 0;
                    for(int j=0; j<10; j++) {
                        if (i > j && squared[(i-j+BUFFSIZE)%BUFFSIZE] > currentSlope) {
                            currentSlope = squared[(i-j+BUFFSIZE)%BUFFSIZE];
                        }
                    }
                    if (currentSlope < lastSlope / 2) continue;

                    outputSignal[i % BUFFSIZE] = true;
                    
                    spk_i = 0.25 * integral[search_ptr] + 0.75 * spk_i;
                    spk_f = 0.25 * highpass[search_ptr] + 0.75 * spk_f;
                    
                    update_rr_intervals(i - lastQRS);
                    lastQRS = i;
                    lastSlope = currentSlope;
                    
                    break;
                }
            }
        }
    }
    
    // --- 动态阈值更新 ---
    threshold_i1 = npk_i + 0.25 * (spk_i - npk_i);
    threshold_f1 = npk_f + 0.25 * (spk_f - npk_f);
    threshold_i2 = 0.5 * threshold_i1;
    threshold_f2 = 0.5 * threshold_f1;
    
    // --- 更新输出缓冲区 ---
    if (qrs) {
        outputSignal[current_ptr] = true;
    } else {
        outputSignal[current_ptr] = false;
    }
    
    sample++;
}

float panTompkins_get_heart_rate(void) {
    if (rravg1 > 0) {
        return current_bpm;
    }
    return 0.0f;
}

int panTompkins_get_detection(long unsigned int *detected_sample_index) {
    if (sample < DELAY) {
        return 0;
    }
    
    int read_ptr = (sample - DELAY + BUFFSIZE) % BUFFSIZE;
    
    if (outputSignal[read_ptr]) {
        outputSignal[read_ptr] = false;
        if (detected_sample_index != NULL) {
            *detected_sample_index = sample - DELAY;
        }
        return 1;
    }
    
    return 0;
}