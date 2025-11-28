#ifndef MOTOR_API_HPP
#define MOTOR_API_HPP

/**
 * @file motor_api.hpp
 * @brief EtherCAT电机控制API头文件
 * 
 * 该文件定义了MotorApi类，提供对多个EtherCAT伺服电机的统一控制接口。
 * 支持自动检测连接的电机数量，并提供位置、速度、力矩控制模式。
 * 通过电机适配器模式支持多厂家电机。
 */

#include <stdint.h>
#include <ecrt.h>
#include <stddef.h>
#include <vector>
#include <memory>
#include "motor_adapter.hpp"

/**
 * @class MotorApi
 * @brief EtherCAT多电机控制API类
 * 
 * 提供对多个EtherCAT伺服电机的统一控制，包括：
 * - 自动检测连接的电机数量
 * - 支持位置、速度、力矩控制模式
 * - 实时状态监控和故障处理
 * - 安全的资源管理和信号处理
 */
class MotorApi {
public:
  /**
   * @brief 构造函数
   * 初始化EtherCAT资源和状态变量
   */
  MotorApi();
  
  /**
   * @brief 析构函数
   * 自动清理EtherCAT资源
   */
  ~MotorApi();
  
  /**
   * @brief 自动初始化EtherCAT主站和从站
   * @return true 初始化成功，false 初始化失败
   * 
   * 自动扫描并配置所有连接的电机从站，注册PDO条目，
   * 激活EtherCAT主站，完成系统初始化。
   */
  bool init_auto();
  
  /**
   * @brief 从ENI文件初始化EtherCAT网络
   * @param eni_filename ENI文件路径
   * @return true 初始化成功，false 初始化失败
   * 
   * 从EtherCAT Network Information (ENI) XML文件导入网络配置，
   * 包括从站信息、PDO映射等，然后激活EtherCAT主站。
   */
  bool init_from_eni(const char* eni_filename);
  
  /**
   * @brief 接收并处理EtherCAT数据
   * 从EtherCAT总线接收最新数据并更新内部状态
   */
  void receive_and_process();
  
  /**
   * @brief 排队并发送EtherCAT数据
   * 将待发送数据排队并通过EtherCAT总线发送
   */
  void queue_and_send();
  
  /**
   * @brief 清理EtherCAT资源
   * 释放EtherCAT主站、域等资源，安全退出
   */
  void cleanup();
  
  /**
   * @brief 信号处理函数
   * @param signum 信号编号
   * 
   * 处理SIGINT等信号，触发清理操作
   */
  static void signal_handler(int);
  
  /**
   * @brief 检查运行状态
   * @return true 运行中，false 已停止
   */
  bool running() const;
  
  /**
   * @brief 获取电机数量
   * @return 检测到的电机数量
   */
  size_t motor_count() const;
  
  /**
   * @brief 设置操作模式
   * @param motor 电机索引
   * @param op_mode 操作模式 (8:位置模式, 9:速度模式, 10:力矩模式)
   * @param resv1_value 保留参数值
   */
  void set_opmode(size_t motor, uint8_t op_mode, uint8_t resv1_value);
  
  /**
   * @brief 获取状态字
   * @param motor 电机索引
   * @return 状态字值
   */
  uint16_t get_status(size_t motor) const;
  
  /**
   * @brief 生成控制字
   * @param motor 电机索引
   * @param status 当前状态字
   * @param start_pos 起始位置引用
   * @param run_enable 运行使能标志引用
   * @return 生成的控制字
   * 
   * 根据当前状态生成适当的控制字，管理状态机转换
   */
  uint16_t make_control(size_t motor, uint16_t status, int32_t &start_pos, bool &run_enable);
  
  /**
   * @brief 写入控制字
   * @param motor 电机索引
   * @param control 控制字值
   */
  void write_control(size_t motor, uint16_t control);
  
  /**
   * @brief 更新目标位置
   * @param motor 电机索引
   * @param pos 目标位置值
   */
  void update_target_pos(size_t motor, int32_t pos);
  
  /**
   * @brief 获取实际位置
   * @param motor 电机索引
   * @return 实际位置值
   */
  int32_t get_actual_pos(size_t motor) const;
  
 /**
   * @brief 复位电机
   * @param motor 电机索引
   * 
   * 发送复位命令（控制字0x0080）到指定电机
   * 控制字0x0080会触发电机驱动器的故障复位
   */
  void reset(size_t motor);

  /**
   * @brief 获取电机适配器信息
   * @param motor 电机索引
   * @return 电机适配器名称
   * 
   * 获取指定电机的适配器名称，用于调试和日志记录
   */
  std::string get_adapter_name(size_t motor) const;

  /**
   * @brief 获取电机厂商信息
   * @param motor 电机索引
   * @return 厂商ID和产品代码的字符串表示
   * 
   * 获取指定电机的厂商信息，用于调试和日志记录
   */
  std::string get_motor_info(size_t motor) const;
private:
  ec_master_t *master_;                    ///< EtherCAT主站句柄
  ec_domain_t *domain_;                      ///< EtherCAT域句柄
  std::vector<ec_slave_config_t*> scs_;      ///< 从站配置数组
  uint8_t *domain_pd_;                       ///< 域过程数据指针
  size_t slave_count_;                       ///< 检测到的从站数量
  std::vector<uint16_t> slave_pos_;          ///< 从站位置索引数组
  
  // 电机适配器数组，每个电机对应一个适配器
  std::vector<std::shared_ptr<MotorAdapter>> motor_adapters_;  ///< 电机适配器
  
  // PDO条目偏移量数组，每个电机对应一组偏移量
  std::vector<std::vector<unsigned int>> pdo_offsets_;  ///< 每个电机的PDO偏移量数组
  
  std::vector<ec_pdo_entry_reg_t> regs_;            ///< PDO条目注册数组
  bool run_;                                        ///< 运行状态标志
};

#endif