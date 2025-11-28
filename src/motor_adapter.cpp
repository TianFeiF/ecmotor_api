#include "motor_adapter.hpp"
#include <cstring>
#include <algorithm>

// MotorAdapterManager实现
MotorAdapterManager& MotorAdapterManager::getInstance() {
    static MotorAdapterManager instance;
    return instance;
}

void MotorAdapterManager::registerAdapter(std::shared_ptr<MotorAdapter> adapter) {
    adapters_.push_back(adapter);
}

std::shared_ptr<MotorAdapter> MotorAdapterManager::findAdapter(uint32_t vendor_id, uint32_t product_code) const {
    for (const auto& adapter : adapters_) {
        if (adapter->supportsMotor(vendor_id, product_code)) {
            return adapter;
        }
    }
    return nullptr;
}

std::vector<std::shared_ptr<MotorAdapter>> MotorAdapterManager::getAllAdapters() const {
    return adapters_;
}

void MotorAdapterManager::clear() {
    adapters_.clear();
}

// StandardMotorAdapter实现
MotorAdapter::MotorInfo StandardMotorAdapter::getMotorInfo() const {
    return {
        0x00000000,  // 默认厂商ID，子类需要重写
        0x00000000,  // 默认产品代码，子类需要重写
        0x00000000,  // 默认修订版本号
        0x00000000,  // 默认序列号
        "Standard Motor",  // 默认名称，子类需要重写
        true,        // 默认支持DC
        -1          // 默认位置，需要设置
    };
}

bool StandardMotorAdapter::configurePdo(ec_slave_config_t* slave_config) {
    // 标准CiA 402 PDO配置
    static ec_pdo_entry_info_t slave_pdo_entries[] = {
        {0x6040, 0x00, 16}, // Control word
        {0x607A, 0x00, 32}, // Target position
        {0x60FF, 0x00, 32}, // Target velocity
        {0x6071, 0x00, 16}, // Target torque
        {0x6060, 0x00, 8},  // Operation mode
        {0x60C2, 0x00, 8},  // Reserved 1
        {0x6041, 0x00, 16}, // Status word
        {0x6064, 0x00, 32}, // Actual position
        {0x606C, 0x00, 32}, // Actual velocity
        {0x6077, 0x00, 16}, // Actual torque
        {0x6061, 0x00, 8},  // Operation mode display
        {0x603F, 0x00, 16}, // Error code
        {0x2026, 0x00, 8},  // Reserved 2
    };

    static ec_pdo_info_t slave_rx_pdo[] = {
        {0x1600, 6, slave_pdo_entries},
    };

    static ec_pdo_info_t slave_tx_pdo[] = {
        {0x1A00, 7, &slave_pdo_entries[6]},
    };

    static ec_sync_info_t slave_syncs[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, slave_rx_pdo, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, slave_tx_pdo, EC_WD_DISABLE},
        {0xff}
    };

    return ecrt_slave_config_pdos(slave_config, EC_END, slave_syncs) == 0;
}

std::vector<MotorAdapter::PdoConfig> StandardMotorAdapter::getRxPdoConfig() const {
    return {
        {0x6040, 0x00, 16}, // Control word
        {0x607A, 0x00, 32}, // Target position
        {0x60FF, 0x00, 32}, // Target velocity
        {0x6071, 0x00, 16}, // Target torque
        {0x6060, 0x00, 8},  // Operation mode
        {0x0000, 0x00, 0},  // Gap (填充)
        {0x0000, 0x00, 0},  // Gap (填充)
        {0x0000, 0x00, 0},  // Gap (填充)
        {0x0000, 0x00, 0},  // Gap (填充)
        {0x0000, 0x00, 0}   // Gap (填充)
    };
}

std::vector<MotorAdapter::PdoConfig> StandardMotorAdapter::getTxPdoConfig() const {
    return {
        {0x6041, 0x00, 16}, // Status word
        {0x6064, 0x00, 32}, // Actual position
        {0x606C, 0x00, 32}, // Actual velocity
        {0x6077, 0x00, 16}, // Actual torque
        {0x6061, 0x00, 8},  // Operation mode display
        {0x603F, 0x00, 16}, // Error code
        {0x0000, 0x00, 0},  // Gap (填充)
        {0x0000, 0x00, 0},  // Gap (填充)
        {0x0000, 0x00, 0},  // Gap (填充)
        {0x0000, 0x00, 0}   // Gap (填充)
    };
}

MotorAdapter::MotorStatus StandardMotorAdapter::readStatus(const uint8_t* domain_pd, 
                                                          const std::vector<unsigned int>& offset) const {
    MotorStatus status = {};
    
    if (offset.size() >= 10) {
        status.status_word = readInt16(domain_pd + offset[0]);
        status.actual_position = readInt32(domain_pd + offset[1]);
        status.actual_velocity = readInt32(domain_pd + offset[2]);
        status.actual_torque = readInt16(domain_pd + offset[3]);
        status.operation_mode = *(domain_pd + offset[4]);
        status.error_code = readInt16(domain_pd + offset[5]);
    }
    
    return status;
}

void StandardMotorAdapter::writeControl(uint8_t* domain_pd, const std::vector<unsigned int>& offset, 
                                      const MotorControl& control) const {
    if (offset.size() >= 10) {
        writeInt16(domain_pd + offset[0], control.control_word);
        writeInt32(domain_pd + offset[1], control.target_position);
        writeInt32(domain_pd + offset[2], control.target_velocity);
        writeInt16(domain_pd + offset[3], control.target_torque);
        *(domain_pd + offset[4]) = control.operation_mode;
        // 剩余的6个偏移量用于Gap填充，不需要写入数据
    }
}

uint16_t StandardMotorAdapter::generateControlWord(uint16_t current_status, bool target_enabled) const {
    // 标准CiA 402状态机控制字生成
    if (target_enabled) {
        // 使能电机
        if ((current_status & 0x004F) == 0x0040) {  // 准备接通状态
            return 0x0006;  // 接通
        } else if ((current_status & 0x006F) == 0x0021) {  // 接通状态
            return 0x000F;  // 使能操作
        } else if ((current_status & 0x006F) == 0x0027) {  // 操作使能状态
            return 0x000F;  // 保持使能
        }
    } else {
        // 禁用电机
        if ((current_status & 0x006F) == 0x0027) {  // 操作使能状态
            return 0x0007;  // 禁用电压
        } else if ((current_status & 0x006F) == 0x0023) {  // 伺服准备好状态
            return 0x0006;  // 快速停止
        } else if ((current_status & 0x006F) == 0x0021) {  // 接通状态
            return 0x0000;  // 准备接通
        }
    }
    return 0x0000;
}

bool StandardMotorAdapter::supportsMotor(uint32_t vendor_id, uint32_t product_code) const {
    // 基类不支持任何特定电机，子类需要重写
    return false;
}

std::string StandardMotorAdapter::getName() const {
    return "Standard Motor Adapter";
}

uint16_t StandardMotorAdapter::makeControl(uint16_t status, int32_t &start_pos, bool &run_enable) const {
  uint16_t control = 0;
  
  // 解析状态字的关键位
  bool ready_to_switch_on = (status & 0x0001) != 0;
  bool switched_on = (status & 0x0002) != 0; 
  bool operation_enabled = (status & 0x0004) != 0;
  bool fault = (status & 0x0008) != 0;
  bool voltage_enabled = (status & 0x0010) != 0;
  bool quick_stop = (status & 0x0020) != 0;
  bool switch_on_disabled = (status & 0x0040) != 0;
  bool warning = (status & 0x0080) != 0;
  
  // 更智能的状态机处理 - 严格按照CiA 402标准
  if (fault) {
    // 故障状态 - 复位故障
    control = 0x0080; // 故障复位
    run_enable = false;
  } else if (warning) {
    // 警告状态 - 先尝试清除警告，然后正常启动流程
    control = 0x0006; // 准备开启
    run_enable = true;
  } else if (switch_on_disabled) {
    // 开关被禁用 - 清除禁用状态
    control = 0x06; // 开启使能（清除禁用）
    run_enable = true; // 准备下一步启用操作
  } else if (quick_stop) {
    // 快速停止状态 - 先禁用快速停止
    control = 0x02; // 禁用快速停止
    run_enable = true; // 准备下一步启用操作
  } else if (!ready_to_switch_on && !switched_on) {
    // 完全关闭状态 - 准备开启
    control = 0x06; // 开启
    run_enable = true;
  } else if (ready_to_switch_on && !switched_on) {
    // 准备好但未开启 - 开启
    control = 0x07; // 开启操作
    run_enable = true;
  } else if (ready_to_switch_on && switched_on && !operation_enabled) {
    // 关键状态：已准备好且已开启，但操作未启用
    control = 0x0F; // 启用操作（这是关键！）
    run_enable = true;
  } else if (operation_enabled) {
    // 操作已启用 - 保持运行状态
    control = 0x0F; // 保持操作
    run_enable = true;
  } else {
    // 默认情况 - 尝试启用操作
    control = 0x06; // 准备开启
    run_enable = true;
  }
  
  return control;
}

// 工具函数实现
int32_t StandardMotorAdapter::readInt32(const uint8_t* data) {
    return static_cast<int32_t>(
        (data[0]) |
        (data[1] << 8) |
        (data[2] << 16) |
        (data[3] << 24));
}

int16_t StandardMotorAdapter::readInt16(const uint8_t* data) {
    return static_cast<int16_t>(
        (data[0]) |
        (data[1] << 8));
}

void StandardMotorAdapter::writeInt32(uint8_t* data, int32_t value) {
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 24) & 0xFF;
}

void StandardMotorAdapter::writeInt16(uint8_t* data, int16_t value) {
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}