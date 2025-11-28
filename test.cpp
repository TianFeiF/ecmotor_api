/**
 * @file test.cpp
 * @brief EtherCAT多电机控制测试程序
 * 
 * 该程序演示了如何使用MotorApi类控制多个EtherCAT伺服电机。
 * 支持自动检测连接的电机数量，执行位置控制循环，
 * 并通过信号处理实现安全退出。
 * 
 * 新特性：
 * - 多厂家电机支持（通过适配器模式）
 * - 自动识别电机型号和适配器
 * - 统一的控制接口，无需关心厂家差异
 */

#include <ecrt.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include "motor_api.hpp"

/**
 * @brief 主函数
 * @return 0 成功，-1 失败
 * 
 * 执行以下操作：
 * 1. 创建MotorApi实例
 * 2. 初始化EtherCAT系统（自动检测电机）
 * 3. 复位所有电机
 * 4. 进入主控制循环
 * 5. 清理资源并退出
 */
int main() {
    // 创建MotorApi实例
    MotorApi api;
    
    // 注册信号处理函数（已由init_auto()内部处理）
    signal(SIGINT, [](int){ /* cleanup handled elsewhere */ });
    
    // 初始化EtherCAT系统
    printf("Initializing EtherCAT system...\n");
    if (!api.init_auto()) {
        printf("Failed to initialize EtherCAT system\n");
        return -1;
    }
    
    // 获取检测到的电机数量
    size_t motor_count = api.motor_count();
    printf("Detected %zu motors\n", motor_count);
    
    // 显示每个电机的适配器信息（新API功能）
    for (size_t m = 0; m < motor_count; ++m) {
        printf("Motor %zu: Adapter=%s, Info=%s\n", 
               m, 
               api.get_adapter_name(m).c_str(),
               api.get_motor_info(m).c_str());
    }
    
    // 配置参数
    int step[] = {500, 500};                // 减少步进值，避免位置跟随错误
    bool run_enable[] = {false, false};   // 运行使能标志
    int32_t start_pos[] = {0, 0};         // 起始位置
    int32_t actual_pos[] = {0, 0};        // 实际位置（用于调试）
    uint8_t op = 8;                       // 操作模式：8=位置模式
    uint8_t tmp = 1;                      // 保留参数
    
    // 复位所有电机
    printf("Resetting all motors...\n");
    for (size_t m = 0; m < motor_count; ++m) {
        api.reset(m);
    }
    
    // 等待复位完成
    usleep(1000000);  // 100ms
    
    printf("Starting control loop...\n");
     int i = 0;
    // 主控制循环
    while (api.running()) {
        // 设置所有电机的操作模式
        for (size_t m = 0; m < motor_count; ++m) {
            api.set_opmode(m, op, tmp);
        }
        
        // 接收并处理EtherCAT数据
        api.receive_and_process();
        
        // 处理每个电机的状态和控制
        for (size_t m = 0; m < motor_count; ++m) {
            // 获取当前状态
            uint16_t status = api.get_status(m);
            actual_pos[m] = api.get_actual_pos(m);
            
            // 生成控制字
            uint16_t control = api.make_control(m, status, start_pos[m], run_enable[m]);
            
            // 写入控制字
            api.write_control(m, control);
        }
        
        // 更新运行中电机的目标位置
        bool any_motor_running = false;
        
        for (size_t m = 0; m < motor_count; ++m) {
            if (run_enable[m]) {
                // 如果是第一次启动，初始化目标位置为当前实际位置
                if (start_pos[m] == 0) {
                    start_pos[m] = actual_pos[m];
                    printf("Motor %zu: Initializing target position to actual position: %d\n", m, start_pos[m]);
                }
                
                start_pos[m] += step[m];
                api.update_target_pos(m, start_pos[m]);
                any_motor_running = true;
                
                if (i == 1000) {
                    printf("Motor %zu: Target=%d, Actual=%d, Status=0x%04X, RunEnable=%d\n", 
                           m, start_pos[m], actual_pos[m], api.get_status(m), run_enable[m]);  
                    i = 0;
                } else {
                    i++;
                }
            } else {
                // 即使电机未运行，也定期打印状态信息用于调试
                if (i == 2000) {
                    printf("Motor %zu: Status=0x%04X, RunEnable=%d, Actual=%d\n", 
                           m, api.get_status(m), run_enable[m], actual_pos[m]);
                    i = 0;
                } else {
                    i++;
                }
            }
        }
        
        // 如果没有电机在运行，显示状态信息帮助调试，并尝试启动电机
        if (!any_motor_running) {
            if (i == 500) {
                printf("No motors running - trying to start motors...\n");
                for (size_t m = 0; m < motor_count; ++m) {
                    uint16_t status = api.get_status(m);
                    printf("  Motor %zu: Status=0x%04X, RunEnable=%d, Actual=%d\n", 
                           m, status, run_enable[m], actual_pos[m]);
                    
                    // 获取当前状态并重新生成控制字，让makeControl决定run_enable
                    uint16_t control = api.make_control(m, status, start_pos[m], run_enable[m]);
                    printf("  Generated control=0x%04X for motor %zu\n", control, m);
                }
                i = 0;
            } else {
                i++;
            }
        }
        
        // 发送EtherCAT数据
        api.queue_and_send();
        
        // 1ms控制周期
        usleep(1000);
    }
    
    printf("Control loop terminated, cleaning up...\n");
    api.cleanup();
    
    printf("Program completed successfully\n");
    return 0;
}