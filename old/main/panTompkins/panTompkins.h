#ifndef PAN_TOMPKINS_H
#define PAN_TOMPKINS_H

#include <stdbool.h>

// ------------------------------------------------------------------------------
// panTompkins.h (流式处理 + 隐式训练 + 回溯搜索版本)
// ------------------------------------------------------------------------------

// 定义输入信号的数据类型，可以根据需要更改为 float, double, int16_t 等
typedef int dataType;

/**
 * @brief 初始化Pan-Tompkins算法的所有状态变量。
 *        在开始处理任何信号之前必须调用此函数。
 */
void panTompkins_init(void);

/**
 * @brief 处理一个新的心电信号样本。
 *        这是算法的核心，应该在每个采样周期调用一次。
 * @param new_sample 新的ECG采样点。
 */
void panTompkins_process(dataType new_sample);

/**
 * @brief 获取当前计算出的心率（BPM）。
 * @return 返回每分钟心跳数（BPM）。如果数据不足，返回0.0f。
 */
float panTompkins_get_heart_rate(void);

/**
 * @brief 检查是否有延迟后的QRS波检测结果输出。
 *        由于算法包含回溯逻辑，检测结果会延迟输出以确保准确性。
 * @param detected_sample_index 如果检测到R峰，此指针将用于返回该峰值的样本索引。
 * @return 如果有新的R峰被检测到，返回1；否则返回0。
 */
int panTompkins_get_detection(long unsigned int *detected_sample_index);


#endif // PAN_TOMPKINS_H