/**
 * @file esp_hal_inline_current_sense.cpp
 */

#include "esp_hal_inline_current_sense.h"
#include "esp_log.h"
#include "esp_check.h"

// 引入 Arduino 兼容层的一些数学工具 (SimpleFOC 需要 _sign 等宏)
#include "common/foc_utils.h" 
#include "common/time_utils.h"

static const char *TAG = "ESP_CS";

// 构造函数
InlineCurrentSenseESP32::InlineCurrentSenseESP32(float shunt_resistor, float gain, int pinA, int pinB, int pinC) 
    : CurrentSense() {

    // 保存本地配置副本
    this->m_shunt_resistor = shunt_resistor;
    this->m_amp_gain = gain;

    // 保存引脚
    this->m_pinA = pinA;
    this->m_pinB = pinB;
    this->m_pinC = pinC;
    this->has_c_phase = (pinC != NOT_SET);

    // 初始化增益与偏移默认值
    gain_a = gain_b = gain_c = 0.0f;
    offset_ia = offset_ib = offset_ic = 0.0f;

    // 初始化指针
    this->adc_handle = NULL;
    for(int i=0; i<3; i++) this->cali_handle[i] = NULL;

}

// 计算增益系数
void InlineCurrentSenseESP32::configureGain() {
    // 伏特转安培的比例: Volts * (1/gain) / R = Amps
    if (m_shunt_resistor != 0 && m_amp_gain != 0) {
        volts_to_amps_ratio = 1.0f / m_shunt_resistor / m_amp_gain;
    } else {
        volts_to_amps_ratio = 0.0f;
    }
    
    // 设置 CurrentSense 基类的增益参数
    // Inline 检测通常增益是对称的，且反相（取决于运放电路，通常是正向）
    // 如果读出来的电流反了，可以在这里加负号
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
}
// 初始化函数
int InlineCurrentSenseESP32::init() {
    ESP_LOGI(TAG, "Initializing InlineCurrentSenseESP32 (Standalone)...");
    
    // 1. 计算增益
    configureGain();

    // 2. 初始化 ADC 硬件
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        // .clk_src = 0 // 默认
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &adc_handle) != ESP_OK) {
        ESP_LOGE(TAG, "ADC Unit Init Failed");
        return 0;
    }
    // 2. 辅助 lambda：配置通道和校准
    auto config_channel = [&](int pin, adc_channel_t &channel, int index) -> bool {
        adc_unit_t unit; // 用于检查引脚是否属于 ADC1
        if (adc_oneshot_io_to_channel(pin, &unit, &channel) != ESP_OK || unit != ADC_UNIT_1) {
            ESP_LOGE(TAG, "Pin %d invalid for ADC1", pin);
            return false;
        }
        // 配置通道参数：12位宽，11dB 衰减 (量程约 0-3.3V)
        adc_oneshot_chan_cfg_t config = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &config));

        // 配置校准 (Curve Fitting 方案)
        ESP_LOGI(TAG, "Calibrating ADC Channel %d...", channel);
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = channel,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        // 尝试创建校准方案，如果失败可能需要检查 eFuse 或 menuconfig
        // adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle[index]);
        if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle[index]) != ESP_OK) {
            ESP_LOGW(TAG, "Calibration failed for channel %d, using raw data", channel);
        }
        return true;
    };

    // 3. 配置各相
    // 【修改点3】：使用本地保存的 m_pinX
    if (!config_channel(m_pinA, channel_a, 0)) return 0;
    if (!config_channel(m_pinB, channel_b, 1)) return 0;
    if (has_c_phase && !config_channel(m_pinC, channel_c, 2)) return 0;
    ESP_LOGI(TAG, "ADC Init & Calibration Success");

    // 3. 校准零点偏移 (调用基类方法)
    ESP_LOGI(TAG, "Calibrating Offsets...");
    calibrateOffsets(); // 本地实现：读取通道并计算平均偏移
    
    ESP_LOGI(TAG, "Init Success. Offsets: %.2f / %.2f / %.2f", offset_ia, offset_ib, offset_ic);
    return 1;
}

// 本地实现的校准函数（因为基类的 calibrateOffsets 是 private）
void InlineCurrentSenseESP32::calibrateOffsets(){
    const int calibration_rounds = 1000;
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;

    for (int i = 0; i < calibration_rounds; i++) {
        if (m_pinA != NOT_SET) offset_ia += readChannelVoltage(channel_a, cali_handle[0]);
        if (m_pinB != NOT_SET) offset_ib += readChannelVoltage(channel_b, cali_handle[1]);
        if (m_pinC != NOT_SET && has_c_phase) offset_ic += readChannelVoltage(channel_c, cali_handle[2]);
        _delay(1);
    }

    if (m_pinA != NOT_SET) offset_ia = offset_ia / calibration_rounds;
    if (m_pinB != NOT_SET) offset_ib = offset_ib / calibration_rounds;
    if (m_pinC != NOT_SET && has_c_phase) offset_ic = offset_ic / calibration_rounds;
}

// 内部辅助：读取电压 (保持不变)
float InlineCurrentSenseESP32::readChannelVoltage(adc_channel_t channel, adc_cali_handle_t cali_handle) {
    int raw = 0;
    // 读取原始值
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &raw));
    
    int voltage_mv = 0;
    if (cali_handle) {
        // 使用校准值转换 (Raw -> mV)
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv));
        return voltage_mv / 1000.0f; // mV -> V
    } else {
        // 无校准回退：Raw / 4095 * 3.3V
        return (raw / 4095.0f) * 3.3f;
    }
}

// 重写 getPhaseCurrents (保持不变)
PhaseCurrent_s InlineCurrentSenseESP32::getPhaseCurrents() {
    PhaseCurrent_s current;

    // 1. 读取电压
    float vol_a = readChannelVoltage(channel_a, cali_handle[0]);
    float vol_b = readChannelVoltage(channel_b, cali_handle[1]);
    float vol_c = 0;
    if(has_c_phase) vol_c = readChannelVoltage(channel_c, cali_handle[2]);

    // 2. 计算电流：(电压 - 零点偏移) * 增益
    // 公式： I = (Voltage - Offset) * Gain
    current.a = (vol_a - offset_ia) * gain_a; 
    current.b = (vol_b - offset_ib) * gain_b;
    
    if(has_c_phase) {
        current.c = (vol_c - offset_ic) * gain_c;
    } else {
        // 如果只有两相，根据基尔霍夫定律 Ia + Ib + Ic = 0 => Ic = -(Ia + Ib)
        current.c = -(current.a + current.b);
    }

    return current;
}

// === 移植自 SimpleFOC InlineCurrentSense.cpp 的 driverAlign ===
// 用于检测电流检测的方向是否与驱动方向一致
int InlineCurrentSenseESP32::driverAlign(float align_voltage) {
    
    if(skip_align) return 1;

    if (!driver || !driver->initialized) {
        ESP_LOGE(TAG, "Driver not linked or init");
        return 0;
    }

    ESP_LOGI(TAG, "Starting Driver Align...");
    
    // 1. 设置相位A电压
    driver->setPwm(align_voltage, 0, 0);
    _delay(200); // 这里的 _delay 是 SimpleFOC 提供的
    
    PhaseCurrent_s c = getPhaseCurrents();
    
    // 检查 A 相电流是否为正 (电压施加在 A，电流应流出 A)
    // 如果小于某个负阈值，说明增益反了
    if (c.a < -0.1f) gain_a *= -1;
    else if (c.a < 0.1f) {
        // 电流太小，可能未连接或电压不够
        ESP_LOGW(TAG, "Current too low for alignment: %.2f", c.a);
        return 0;
    }

    // 2. 设置相位B
    driver->setPwm(0, align_voltage, 0);
    _delay(200);
    c = getPhaseCurrents();
    if (c.b < -0.1f) gain_b *= -1;
    else if (c.b < 0.1f) return 0;
    
    // 3. 设置相位C (如有)
    if(has_c_phase) {
        driver->setPwm(0, 0, align_voltage);
        _delay(200);
        c = getPhaseCurrents();
        if (c.c < -0.1f) gain_c *= -1;
        else if (c.c < 0.1f) return 0;
    }

    // 恢复 0 输出
    driver->setPwm(0, 0, 0);
    ESP_LOGI(TAG, "Driver Align Success");
    return 1;
}