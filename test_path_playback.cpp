#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <signal.h>
#include <iomanip>
#include "motor_api.hpp"

// 路径数据结构
struct PathPoint {
    double position;    // 位置（度）
    double time_ms;     // 时间（毫秒）
};

class PathPlayer {
private:
    std::vector<PathPoint> path_data_;
    size_t current_index_ = 0;
    double start_time_ms_ = 0;
    bool is_playing_ = false;
    
public:
    // 从CSV文件加载路径数据
    bool loadPath(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "无法打开路径文件: " << filename << std::endl;
            return false;
        }
        
        path_data_.clear();
        std::string line;
        int line_count = 0;
        double dt = 8.0; // 默认8ms间隔
        
        while (std::getline(file, line)) {
            line_count++;
            
            // 跳过注释行
            if (line.empty() || line[0] == '#') continue;
            
            try {
                double position = std::stod(line);
                double time_ms = (path_data_.size()) * dt; // 计算时间点
                
                path_data_.push_back({position, time_ms});
            } catch (const std::exception& e) {
                std::cerr << "解析行 " << line_count << " 失败: " << e.what() << std::endl;
                continue;
            }
        }
        
        std::cout << "成功加载 " << path_data_.size() << " 个路径点" << std::endl;
        return !path_data_.empty();
    }
    
    // 开始播放路径
    void startPlayback() {
        if (path_data_.empty()) {
            std::cerr << "路径数据为空，无法播放" << std::endl;
            return;
        }
        
        current_index_ = 0;
        start_time_ms_ = getCurrentTimeMs();
        is_playing_ = true;
        
        std::cout << "开始路径播放，总点数: " << path_data_.size() << std::endl;
    }
    
    // 获取当前时间（毫秒）
    double getCurrentTimeMs() {
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration<double, std::milli>(duration).count();
    }
    
    // 更新路径播放，返回当前目标位置
    double updatePlayback() {
        if (!is_playing_ || path_data_.empty()) {
            return 0.0;
        }
        
        double current_time = getCurrentTimeMs() - start_time_ms_;
        
        // 查找当前时间点对应的路径点
        while (current_index_ < path_data_.size() - 1 && 
               current_time >= path_data_[current_index_ + 1].time_ms) {
            current_index_++;
        }
        
        // 路径播放完成
        if (current_index_ >= path_data_.size() - 1) {
            is_playing_ = false;
            std::cout << "路径播放完成" << std::endl;
            return path_data_.back().position;
        }
        
        // 在当前点和下一点之间插值
        const PathPoint& current_point = path_data_[current_index_];
        const PathPoint& next_point = path_data_[current_index_ + 1];
        
        double time_ratio = 0.0;
        if (next_point.time_ms > current_point.time_ms) {
            time_ratio = (current_time - current_point.time_ms) / 
                        (next_point.time_ms - current_point.time_ms);
        }
        
        // 线性插值
        double interpolated_position = current_point.position + 
                                     time_ratio * (next_point.position - current_point.position);
        
        return interpolated_position;
    }
    
    // 检查是否正在播放
    bool isPlaying() const { return is_playing_; }
    
    // 停止播放
    void stopPlayback() { is_playing_ = false; }
    
    // 重置到开始位置
    void reset() {
        current_index_ = 0;
        is_playing_ = false;
    }
};

// 全局变量
static volatile bool g_running = true;
static MotorApi* g_api = nullptr;

// 电机参数定义
const double ENCODER_RESOLUTION = 65535.0;  // 16位编码器分辨率 (0-65535)
const double GEAR_RATIO = 101.0;          // 减速比 101:1
const double MOTOR_UNITS_PER_DEG = ENCODER_RESOLUTION * GEAR_RATIO / 360.0;  // 电机单位/度

void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "\n收到中断信号，正在停止..." << std::endl;
        g_running = false;
    }
}

int main(int argc, char* argv[]) {
    // 设置信号处理
    signal(SIGINT, signal_handler);
    
    // 创建电机API实例
    MotorApi api;
    g_api = &api;
    
    // 路径文件参数
    std::string path_file = "path_example_deg.csv";
    if (argc > 1) {
        path_file = argv[1];
    }
    
    std::cout << "=== EtherCAT路径播放测试 ===" << std::endl;
    std::cout << "路径文件: " << path_file << std::endl;
    
    // 初始化EtherCAT系统（使用自动检测模式）
    std::cout << "初始化EtherCAT系统..." << std::endl;
    if (!api.init_auto()) {
        std::cerr << "EtherCAT初始化失败" << std::endl;
        return -1;
    }
    
    std::cout << "检测到 " << api.motor_count() << " 个从站" << std::endl;
    
    // 创建路径播放器
    PathPlayer path_player;
    
    // 加载路径数据
    if (!path_player.loadPath(path_file)) {
        std::cerr << "加载路径文件失败" << std::endl;
        return -1;
    }
    
    // 启动路径播放
    path_player.startPlayback();
    
    // 控制循环
    const double dt = 0.008; // 8ms控制周期，与路径数据匹配
    const int control_hz = static_cast<int>(1.0 / dt);
    
    std::cout << "开始路径跟踪控制 (" << control_hz << " Hz)..." << std::endl;
    std::cout << "按Ctrl+C停止" << std::endl;
    
    // 设置所有电机为位置模式
    for (int m = 0; m < api.motor_count(); m++) {
        api.set_opmode(m, 0x08, 0); // 位置模式
    }
    
    int loop_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    while (g_running && path_player.isPlaying()) {
        auto loop_start = std::chrono::steady_clock::now();
        
        // 更新路径播放，获取目标位置
        double target_position_deg = path_player.updatePlayback();
        
        // 将角度转换为电机单位（考虑编码器分辨率和减速比）
        int32_t target_position = static_cast<int32_t>(target_position_deg * MOTOR_UNITS_PER_DEG);
        
        // 设置所有电机的目标位置
        for (int m = 0; m < api.motor_count(); m++) {
            api.update_target_pos(m, target_position);
        }
        
        // 发送控制命令
        for (int m = 0; m < api.motor_count(); m++) {
            uint16_t status = api.get_status(m);
            int32_t start_pos = 0;
            bool run_enable = true;  // 始终尝试保持运行状态
            uint16_t control = api.make_control(m, status, start_pos, run_enable); // 生成控制字
            api.write_control(m, control);
        }
        
        // 处理EtherCAT通信
        api.receive_and_process();
        api.queue_and_send();
        
        // 显示状态信息（每100个循环显示一次）
        if (loop_count % 100 == 0) {
            std::cout << "目标位置: " << std::fixed << std::setprecision(2) 
                     << target_position_deg << "°";
            
            for (int m = 0; m < api.motor_count(); m++) {
                int32_t actual_pos = api.get_actual_pos(m);
                double actual_deg = actual_pos * 360.0 / (ENCODER_RESOLUTION * GEAR_RATIO);
                uint16_t status = api.get_status(m);
                std::cout << " | 电机" << m << ": " << std::fixed << std::setprecision(2) 
                         << actual_deg << "° (状态: 0x" << std::hex << status << std::dec << ")";
            }
            std::cout << std::endl;
        }
        
        loop_count++;
        
        // 精确控制循环周期
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(loop_end - loop_start).count();
        
        if (elapsed < dt) {
            std::this_thread::sleep_for(std::chrono::duration<double>(dt - elapsed));
        }
    }
    
    // 停止所有电机
    std::cout << "停止所有电机..." << std::endl;
    for (int m = 0; m < api.motor_count(); m++) {
        uint16_t status = api.get_status(m);
        int32_t start_pos = 0;
        bool run_enable = false;
        uint16_t control = api.make_control(m, status, start_pos, run_enable); // 停止电机
        api.write_control(m, control);
    }
    
    api.receive_and_process();
    api.queue_and_send();
    
    auto total_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    std::cout << "路径播放完成，总耗时: " << std::fixed << std::setprecision(2) 
              << total_time << " 秒" << std::endl;
    std::cout << "总循环次数: " << loop_count << std::endl;
    
    return 0;
}