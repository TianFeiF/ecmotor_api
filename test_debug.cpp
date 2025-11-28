/**
 * @file test_debug.cpp
 * @brief EtherCAT多电机控制调试程序
 * 
 * 该程序用于调试多电机控制问题，显示详细的状态信息
 */

#include <ecrt.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include "motor_api.hpp"

static bool g_running = true;

void signal_handler(int sig) {
    printf("\nReceived signal %d, shutting down...\n", sig);
    g_running = false;
}

int main() {
    // 创建MotorApi实例
    MotorApi api;
    
    // 注册信号处理函数
    signal(SIGINT, signal_handler);
    
    // 初始化EtherCAT系统
    printf("Initializing EtherCAT system...\n");
    if (!api.init_auto()) {
        printf("Failed to initialize EtherCAT system\n");
        return -1;
    }
    
    // 获取检测到的电机数量
    size_t motor_count = api.motor_count();
    printf("Detected %zu motors\n", motor_count);
    
    // 显示每个电机的适配器信息
    for (size_t m = 0; m < motor_count; ++m) {
        printf("Motor %zu: Adapter=%s, Info=%s\n", 
               m, 
               api.get_adapter_name(m).c_str(),
               api.get_motor_info(m).c_str());
    }
    
    // 配置参数
    int step[] = {500, 600};              // 每个电机的步进值
    bool run_enable[] = {false, false};   // 运行使能标志
    int32_t start_pos[] = {0, 0};         // 起始位置
    uint8_t op = 8;                       // 操作模式：8=位置模式
    uint8_t tmp = 1;                      // 保留参数
    
    // 复位所有电机
    printf("Resetting all motors...\n");
    for (size_t m = 0; m < motor_count; ++m) {
        api.reset(m);
    }
    
    // 等待复位完成
    usleep(1000000);  // 1000ms
    
    printf("Starting detailed debug loop...\n");
    int cycle_count = 0;
    
    // 主控制循环 - 限制运行时间用于测试
    int max_cycles = 2000; // 最多运行2000个周期（约2秒）
    while (g_running && api.running() && cycle_count < max_cycles) {
        // 设置所有电机的操作模式
        for (size_t m = 0; m < motor_count; ++m) {
            api.set_opmode(m, op, tmp);
        }
        
        // 接收并处理EtherCAT数据
        api.receive_and_process();
        
        // 每100个周期打印一次详细状态
        if (cycle_count % 100 == 0) {
            printf("\n=== Cycle %d ===\n", cycle_count);
            
            for (size_t m = 0; m < motor_count; ++m) {
                // 获取当前状态
                uint16_t status = api.get_status(m);
                int32_t actual_pos = api.get_actual_pos(m);
                
                printf("Motor %zu: Status=0x%04X, RunEnable=%d, StartPos=%d, ActualPos=%d\n",
                       m, status, run_enable[m], start_pos[m], actual_pos);
                
                // 生成控制字
                uint16_t control = api.make_control(m, status, start_pos[m], run_enable[m]);
                printf("  Control=0x%04X (generated)\n", control);
                
                // 写入控制字
                api.write_control(m, control);
            }
        } else {
            // 正常运行模式（不打印详细信息）
            for (size_t m = 0; m < motor_count; ++m) {
                // 获取当前状态
                uint16_t status = api.get_status(m);
                
                // 生成控制字
                uint16_t control = api.make_control(m, status, start_pos[m], run_enable[m]);
                
                // 写入控制字
                api.write_control(m, control);
            }
        }
        
        // 更新运行中电机的目标位置
        for (size_t m = 0; m < motor_count; ++m) {
            if (run_enable[m]) {
                start_pos[m] += step[m];
                api.update_target_pos(m, start_pos[m]);
                
                // 每1000个周期打印一次位置更新（简化输出）
                if (cycle_count % 1000 == 0 && cycle_count % 100 != 0) {
                    printf("Motor %zu target pos: %d\n", m, start_pos[m]);
                }
            }
        }
        
        // 发送EtherCAT数据
        api.queue_and_send();
        
        // 1ms控制周期
        usleep(1000);
        cycle_count++;
    }
    
    printf("Control loop terminated after %d cycles, cleaning up...\n", cycle_count);
    api.cleanup();
    
    printf("Program completed successfully\n");
    return 0;
}