#ifndef EYOU_MOTOR_ADAPTER_HPP
#define EYOU_MOTOR_ADAPTER_HPP

#include "motor_adapter.hpp"

/**
 * @brief EYOU电机适配器
 * 
 * 适配EYOU品牌的EtherCAT伺服电机
 * 厂商ID: 0x00001097
 * 产品代码: 0x00002406
 */
class EyouMotorAdapter : public StandardMotorAdapter {
public:
    MotorInfo getMotorInfo() const override;
    bool supportsMotor(uint32_t vendor_id, uint32_t product_code) const override;
    std::string getName() const override;
    uint16_t makeControl(uint16_t status, int32_t &start_pos, bool &run_enable) const override;
};

/**
 * @brief Delta电机适配器
 * 
 * 适配Delta品牌的EtherCAT伺服电机
 * 示例厂商ID: 0x00000001 (需要根据实际修改)
 */
class DeltaMotorAdapter : public StandardMotorAdapter {
public:
    MotorInfo getMotorInfo() const override;
    bool supportsMotor(uint32_t vendor_id, uint32_t product_code) const override;
    std::string getName() const override;
    uint16_t makeControl(uint16_t status, int32_t &start_pos, bool &run_enable) const override {
        return StandardMotorAdapter::makeControl(status, start_pos, run_enable);
    }
};

/**
 * @brief Yaskawa电机适配器
 * 
 * 适配Yaskawa品牌的EtherCAT伺服电机
 * 示例厂商ID: 0x00000002 (需要根据实际修改)
 */
class YaskawaMotorAdapter : public StandardMotorAdapter {
public:
    MotorInfo getMotorInfo() const override;
    bool supportsMotor(uint32_t vendor_id, uint32_t product_code) const override;
    std::string getName() const override;
    uint16_t makeControl(uint16_t status, int32_t &start_pos, bool &run_enable) const override {
        return StandardMotorAdapter::makeControl(status, start_pos, run_enable);
    }
};

/**
 * @brief Panasonic电机适配器
 * 
 * 适配Panasonic品牌的EtherCAT伺服电机
 * 示例厂商ID: 0x00000003 (需要根据实际修改)
 */
class PanasonicMotorAdapter : public StandardMotorAdapter {
public:
    MotorInfo getMotorInfo() const override;
    bool supportsMotor(uint32_t vendor_id, uint32_t product_code) const override;
    std::string getName() const override;
    uint16_t makeControl(uint16_t status, int32_t &start_pos, bool &run_enable) const override {
        return StandardMotorAdapter::makeControl(status, start_pos, run_enable);
    }
};

#endif // EYOU_MOTOR_ADAPTER_HPP