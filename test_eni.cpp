/**
 * @file test_eni.cpp
 * @brief EtherCAT多电机控制测试程序 - ENI文件版本
 * 
 * 该程序演示了如何使用MotorApi类通过ENI文件配置控制多个EtherCAT伺服电机。
 * 支持从ENI文件导入从站参数，执行位置控制循环，
 * 并通过信号处理实现安全退出。
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
 * 2. 从ENI文件初始化EtherCAT系统
 * 3. 复位所有电机
 * 4. 进入主控制循环
 * 5. 清理资源并退出
 */
int main() {
    // 创建MotorApi实例
    MotorApi api;
    
    // 注册信号处理函数（已由init_auto()内部处理）
    signal(SIGINT, [](int){ /* cleanup handled elsewhere */ });
    
    // 从ENI文件初始化EtherCAT系统
    printf("Initializing EtherCAT system from ENI file...\n");
    if (!api.init_from_eni("test_eni.txt")) {
        printf("Failed to initialize EtherCAT system from ENI file\n");
        return -1;
    }
    
    // 获取检测到的电机数量
    size_t motor_count = api.motor_count();
    printf("Detected %zu motors from ENI file\n", motor_count);
    
    if (motor_count == 0) {
        printf("No motors detected from ENI file\n");
        return -1;
    }
    
    // 配置参数
    std::vector<int> step(motor_count, 500);              // 每个电机的步进值
    std::vector<bool> run_enable(motor_count, false);     // 运行使能标志
    std::vector<int32_t> start_pos(motor_count, 0);       // 起始位置
    uint8_t op = 8;                                       // 操作模式：8=位置模式
    uint8_t tmp = 1;                                      // 保留参数
    
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
            
            // 获取运行使能状态
            bool current_run_enable = run_enable[m];
            
            // 生成控制字
            uint16_t control = api.make_control(m, status, start_pos[m], current_run_enable);
            
            // 更新运行使能状态
            run_enable[m] = current_run_enable;
            
            // 写入控制字
            api.write_control(m, control);
        }
        
        // 更新运行中电机的目标位置
        for (size_t m = 0; m < motor_count; ++m) {
            if (run_enable[m]) {
                start_pos[m] += step[m];
                api.update_target_pos(m, start_pos[m]);
                if (i == 1000) {
                    printf("Motor %zu target pos: %d\n", m, start_pos[m]);  
                    i = 0;
                } else {
                    i++;
                }
            }
        }
        
        // 发送EtherCAT数据
        api.queue_and_send();
        
        // 1ms控制周期
        usleep(1000);
    }
    
    printf("Control loop terminated, cleaning up...\n");
    api.cleanup();
    
    printf("ENI file test completed successfully\n");
    return 0;
}