#include "vendor_adapters.hpp"
#include <iostream>

// EYOU电机适配器实现
MotorAdapter::MotorInfo EyouMotorAdapter::getMotorInfo() const {
    return {
        0x00001097,     // EYOU厂商ID
        0x00002406,     // EYOU产品代码
        0x00000000,     // 修订版本号
        0x00000000,     // 序列号
        "EYOU Servo Motor",  // 电机名称
        true,           // 支持DC
        -1              // 位置待定
    };
}

bool EyouMotorAdapter::supportsMotor(uint32_t vendor_id, uint32_t product_code) const {
    return (vendor_id == 0x00001097 && product_code == 0x00002406);
}

std::string EyouMotorAdapter::getName() const {
    return "EYOU Motor Adapter";
}

uint16_t EyouMotorAdapter::makeControl(uint16_t status, int32_t &start_pos, bool &run_enable) const {
    static int fault_reset_count = 0;
    static int state_change_delay = 0;
    static uint16_t last_status = 0;
    
    // 如果状态发生变化，增加延迟计数器
    if (status != last_status) {
        state_change_delay = 0;
        last_status = status;
        printf("EYOU Motor: Status changed to 0x%04X, starting delay counter\n", status);
    } else {
        state_change_delay++;
    }
    
    // 在状态变化初期添加延迟，避免过快的状态切换
    if (state_change_delay < 5) {  // 延迟5个周期（5ms）
        printf("EYOU Motor: Delaying state change, counter=%d\n", state_change_delay);
        return 0x0000;  // 保持当前状态
    }
    
    // 解析状态字的关键位
    bool ready_to_switch_on = (status & 0x0001) != 0;
    bool switched_on = (status & 0x0002) != 0; 
    bool operation_enabled = (status & 0x0004) != 0;
    bool fault = (status & 0x0008) != 0;
    bool voltage_enabled = (status & 0x0010) != 0;
    bool quick_stop = (status & 0x0020) != 0;
    bool switch_on_disabled = (status & 0x0040) != 0;
    bool warning = (status & 0x0080) != 0;
    
    // 调试输出
    printf("EYOU Motor: Status=0x%04X, Ready=%d, Switched=%d, OpEnabled=%d, Fault=%d, Warning=%d\n", 
           status, ready_to_switch_on, switched_on, operation_enabled, fault, warning);
    
    // EYOU电机特定的故障处理 - 更智能的故障识别
    if (fault) {
        // 分析故障类型 - 检查高字节中的故障代码
        uint8_t fault_code = (status >> 8) & 0xFF;
        static int fault_reset_count = 0;
        printf("EYOU Motor: Fault detected, code=0x%02X, reset_count=%d\n", fault_code, fault_reset_count);
        
        // 对于位置跟随错误（常见于0xXX08状态），需要特殊处理
        if (fault_code == 0x08 || fault_code == 0x09) {
            printf("EYOU Motor: Position following error detected\n");
            // 位置跟随错误通常需要降低目标位置变化率或重新同步位置
            run_enable = false;
            fault_reset_count = 0;
            return 0x0080; // 故障复位
        }
        
        fault_reset_count++;
        
        if (fault_reset_count < 10) {
            // 前10次尝试进行故障复位
            run_enable = false;
            return 0x0080; // 故障复位
        } else {
            // 超过10次尝试后，尝试强制清除故障
            fault_reset_count = 0;
            run_enable = true;
            printf("EYOU Motor: Force fault clear, attempting restart\n");
            return 0x0006; // 尝试重新启动
        }
    }
    
    // 如果存在警告，先尝试清除警告状态
    if (warning) {
        printf("EYOU Motor: Warning detected, attempting to clear\n");
        run_enable = true;
        // 对于警告状态，根据当前状态决定控制字
        if (ready_to_switch_on && switched_on && !operation_enabled) {
            // 已准备好且已开启，但操作未启用 - 尝试启用操作
            return 0x000F; // 启用操作
        } else if (ready_to_switch_on && !switched_on) {
            // 已准备好但未开启 - 尝试开启
            return 0x0007; // 开启操作
        } else {
            // 其他情况 - 准备开启
            return 0x0006; // 准备开启
        }
    }
    
    // 处理快速停止状态 (0x0230/0x0250)
    if (quick_stop && !fault && !warning) {
        printf("EYOU Motor: Quick stop detected, attempting to clear\n");
        
        // 特殊处理：如果已经准备好但快速停止激活，尝试不同的方法
        if (ready_to_switch_on && !switched_on) {
            printf("EYOU Motor: Ready but quick stop active, trying to switch on first\n");
            run_enable = true;
            return 0x0007; // 先尝试开启操作
        } else if (ready_to_switch_on && switched_on) {
            printf("EYOU Motor: Ready and switched on but quick stop active, disabling quick stop\n");
            run_enable = true;
            return 0x0002; // 禁用快速停止
        } else {
            printf("EYOU Motor: Quick stop with other conditions, standard disable\n");
            run_enable = false; // 停止运行
            return 0x0002; // 禁用快速停止
        }
    }
    
    // EYOU电机特殊处理：状态0x0231需要特殊处理
    if (ready_to_switch_on && !switched_on && !operation_enabled && !fault && !warning && quick_stop) {
        printf("EYOU Motor: State 0x0231 detected (ready but quick stop), trying to switch on\n");
        run_enable = true;
        return 0x0007; // 尝试开启操作，绕过快速停止
    }
    
    // EYOU电机特殊处理：初始状态0x0000需要特殊处理
    if (!ready_to_switch_on && !switched_on && !operation_enabled && !fault && !warning && !quick_stop) {
        printf("EYOU Motor: Initial state 0x0000 detected, sending shutdown command\n");
        run_enable = true; // 准备启动流程
        return 0x0006; // 发送准备开启命令
    }
    
    // EYOU电机特殊处理：状态0x0006需要特殊处理
    if (ready_to_switch_on && switched_on && !operation_enabled && !quick_stop) {
        printf("EYOU Motor: Special handling for status 0x0006, forcing operation enable\n");
        run_enable = true;
        return 0x000F; // 直接启用操作
    }
    
    // 对于其他状态，使用标准的状态机逻辑
    uint16_t result = StandardMotorAdapter::makeControl(status, start_pos, run_enable);
    printf("EYOU Motor: Standard adapter returned 0x%04X, run_enable=%d\n", result, run_enable);
    return result;
}

// Delta电机适配器实现
MotorAdapter::MotorInfo DeltaMotorAdapter::getMotorInfo() const {
    return {
        0x00000001,     // Delta厂商ID (示例，需要确认实际值)
        0x12345678,     // Delta产品代码 (示例，需要确认实际值)
        0x00000000,     // 修订版本号
        0x00000000,     // 序列号
        "Delta Servo Motor",  // 电机名称
        true,           // 支持DC
        -1              // 位置待定
    };
}

bool DeltaMotorAdapter::supportsMotor(uint32_t vendor_id, uint32_t product_code) const {
    // 这里需要替换为实际的Delta电机ID
    return (vendor_id == 0x00000001 && product_code == 0x12345678);
}

std::string DeltaMotorAdapter::getName() const {
    return "Delta Motor Adapter";
}

// Yaskawa电机适配器实现
MotorAdapter::MotorInfo YaskawaMotorAdapter::getMotorInfo() const {
    return {
        0x00000002,     // Yaskawa厂商ID (示例，需要确认实际值)
        0x87654321,     // Yaskawa产品代码 (示例，需要确认实际值)
        0x00000000,     // 修订版本号
        0x00000000,     // 序列号
        "Yaskawa Servo Motor",  // 电机名称
        true,           // 支持DC
        -1              // 位置待定
    };
}

bool YaskawaMotorAdapter::supportsMotor(uint32_t vendor_id, uint32_t product_code) const {
    // 这里需要替换为实际的Yaskawa电机ID
    return (vendor_id == 0x00000002 && product_code == 0x87654321);
}

std::string YaskawaMotorAdapter::getName() const {
    return "Yaskawa Motor Adapter";
}

// Panasonic电机适配器实现
MotorAdapter::MotorInfo PanasonicMotorAdapter::getMotorInfo() const {
    return {
        0x00000003,     // Panasonic厂商ID (示例，需要确认实际值)
        0x11223344,     // Panasonic产品代码 (示例，需要确认实际值)
        0x00000000,     // 修订版本号
        0x00000000,     // 序列号
        "Panasonic Servo Motor",  // 电机名称
        true,           // 支持DC
        -1              // 位置待定
    };
}

bool PanasonicMotorAdapter::supportsMotor(uint32_t vendor_id, uint32_t product_code) const {
    // 这里需要替换为实际的Panasonic电机ID
    return (vendor_id == 0x00000003 && product_code == 0x11223344);
}

std::string PanasonicMotorAdapter::getName() const {
    return "Panasonic Motor Adapter";
}