/**
 * @file esp_hal_inline_current_sense.h
 * @brief ESP32 specific implementation of InlineCurrentSense using ESP-IDF ADC OneShot mode
 */

#ifndef ESP_HAL_INLINE_CURRENT_SENSE_H
#define ESP_HAL_INLINE_CURRENT_SENSE_H

// 1. 改回继承底层的抽象基类 CurrentSense
#include "common/base_classes/CurrentSense.h"
// 引入必要的 ESP-IDF 驱动
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// 2. 继承 InlineCurrentSense（包含 gain/offset/calibrateOffsets 等）
class InlineCurrentSenseESP32 : public CurrentSense {
public:
    /**
     * @brief 构造函数
     * @param shunt_resistor 分流电阻值
     * @param gain 运放增益
     * @param pinA A相电流采样引脚
     * @param pinB B相电流采样引脚
     * @param pinC C相电流采样引脚 (可选)
     */
    InlineCurrentSenseESP32(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);

    /**
     * @brief 初始化 ADC 单元和通道，并进行校准配置
     * @return 1 成功, 0 失败
     */
    int init() override;

    /**
     * @brief 重写读取相电流的方法，使用 ESP-IDF 的 ADC API
     */
    PhaseCurrent_s getPhaseCurrents() override;
    // 原本由 InlineCurrentSense 实现的 driverAlign，现在我们需要自己实现
    int driverAlign(float align_voltage) override;

private:
    // ESP-IDF ADC 句柄
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle[3]; // A, B, C 三个通道的校准句柄
    
    // 保存引脚对应的 ADC 通道号
    adc_channel_t channel_a, channel_b, channel_c;
    
    // 配置参数 (保存为本地变量以便访问)
    float m_shunt_resistor;
    float m_amp_gain;
    float volts_to_amps_ratio;

    // 是否使用 C 相
    bool has_c_phase;

    // 因为父类的 pinA 是 private 的，我们需要自己存一份
    int m_pinA, m_pinB, m_pinC;

    // 为避免依赖 InlineCurrentSense，复制其需要的成员
    float gain_a; //!< phase A gain
    float gain_b; //!< phase B gain
    float gain_c; //!< phase C gain

    float offset_ia; //!< zero current A voltage value
    float offset_ib; //!< zero current B voltage value
    float offset_ic; //!< zero current C voltage value

    // 内部辅助函数
    /**
     * @brief 内部辅助函数：读取指定通道的电压值（经过校准）
     * @param channel ADC 通道
     * @param cali_handle 对应的校准句柄
     * @return 电压值 (Volts)
     */
    float readChannelVoltage(adc_channel_t channel, adc_cali_handle_t cali_handle);
    /**
     * @brief 计算并校准 ADC 零点偏移（在 init 后调用）
     */
    void calibrateOffsets();
    
    // 简单的对齐辅助
    void configureGain();
};

#endif // ESP_HAL_INLINE_CURRENT_SENSE_H