#ifndef MOTOR_ADAPTER_HPP
#define MOTOR_ADAPTER_HPP

#include <ecrt.h>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

/**
 * @brief 电机适配器基类
 * 
 * 提供通用接口，支持不同厂家电机的EtherCAT配置和控制
 * 每个厂家的电机都需要实现对应的适配器
 */
class MotorAdapter {
public:
    /**
     * @brief 电机信息结构体
     */
    struct MotorInfo {
        uint32_t vendor_id;           ///< 厂商ID
        uint32_t product_code;          ///< 产品代码
        uint32_t revision_number;       ///< 修订版本号
        uint32_t serial_number;         ///< 序列号
        std::string name;               ///< 电机名称
        bool has_dc;                    ///< 是否支持分布式时钟
        int position;                   ///< 从站位置
    };

    /**
     * @brief PDO配置结构体
     */
    struct PdoConfig {
        uint16_t index;                 ///< PDO索引
        uint8_t subindex;               ///< 子索引
        uint8_t bit_length;             ///< 位长度
    };

    /**
     * @brief 电机状态结构体
     */
    struct MotorStatus {
        uint16_t status_word;           ///< 状态字
        int32_t actual_position;        ///< 实际位置
        int32_t actual_velocity;        ///< 实际速度
        int16_t actual_torque;          ///< 实际力矩
        uint8_t operation_mode;         ///< 操作模式
        uint16_t error_code;            ///< 错误代码
    };

    /**
     * @brief 电机控制结构体
     */
    struct MotorControl {
        uint16_t control_word;          ///< 控制字
        int32_t target_position;        ///< 目标位置
        int32_t target_velocity;        ///< 目标速度
        int16_t target_torque;          ///< 目标力矩
        uint8_t operation_mode;         ///< 操作模式
    };

    virtual ~MotorAdapter() = default;

    /**
   * @brief 生成控制字
   * @param status 当前状态字
   * @param start_pos 起始位置引用
   * @param run_enable 运行使能标志引用
   * @return 生成的控制字
   * 
   * 根据电机当前状态生成适当的控制字，管理状态机转换
   */
  virtual uint16_t makeControl(uint16_t status, int32_t &start_pos, bool &run_enable) const = 0;

  /**
   * @brief 获取电机信息
   * @return 电机基本信息结构体
   */
  virtual MotorInfo getMotorInfo() const = 0;

    /**
     * @brief 配置PDO映射
     * @param slave_config 从站配置句柄
     * @return true 成功，false 失败
     */
    virtual bool configurePdo(ec_slave_config_t* slave_config) = 0;

    /**
     * @brief 获取RxPDO配置
     * @return RxPDO配置数组
     */
    virtual std::vector<PdoConfig> getRxPdoConfig() const = 0;

    /**
     * @brief 获取TxPDO配置
     * @return TxPDO配置数组
     */
    virtual std::vector<PdoConfig> getTxPdoConfig() const = 0;

    /**
     * @brief 从PDO数据读取电机状态
     * @param domain_pd 域过程数据指针
     * @param offset PDO偏移量数组
     * @return 电机状态
     */
    virtual MotorStatus readStatus(const uint8_t* domain_pd, const std::vector<unsigned int>& offset) const = 0;

    /**
     * @brief 将控制数据写入PDO
     * @param domain_pd 域过程数据指针
     * @param offset PDO偏移量数组
     * @param control 控制数据
     */
    virtual void writeControl(uint8_t* domain_pd, const std::vector<unsigned int>& offset, 
                             const MotorControl& control) const = 0;

    /**
     * @brief 生成控制字
     * @param current_status 当前状态字
     * @param target_enabled 目标使能状态
     * @return 控制字
     */
    virtual uint16_t generateControlWord(uint16_t current_status, bool target_enabled) const = 0;

    /**
     * @brief 检查是否支持该电机
     * @param vendor_id 厂商ID
     * @param product_code 产品代码
     * @return true 支持，false 不支持
     */
    virtual bool supportsMotor(uint32_t vendor_id, uint32_t product_code) const = 0;

    /**
     * @brief 获取适配器名称
     * @return 适配器名称
     */
    virtual std::string getName() const = 0;
};

/**
 * @brief 电机适配器管理器
 * 
 * 管理所有注册的电机适配器，提供统一的电机适配接口
 */
class MotorAdapterManager {
public:
    /**
     * @brief 获取单例实例
     * @return 管理器实例
     */
    static MotorAdapterManager& getInstance();

    /**
     * @brief 注册电机适配器
     * @param adapter 适配器指针
     */
    void registerAdapter(std::shared_ptr<MotorAdapter> adapter);

    /**
     * @brief 根据电机信息查找适配器
     * @param vendor_id 厂商ID
     * @param product_code 产品代码
     * @return 适配器指针，未找到返回nullptr
     */
    std::shared_ptr<MotorAdapter> findAdapter(uint32_t vendor_id, uint32_t product_code) const;

    /**
     * @brief 获取所有注册的适配器
     * @return 适配器列表
     */
    std::vector<std::shared_ptr<MotorAdapter>> getAllAdapters() const;

    /**
     * @brief 清空所有适配器
     */
    void clear();

private:
    MotorAdapterManager() = default;
    std::vector<std::shared_ptr<MotorAdapter>> adapters_;
};

/**
 * @brief 标准EtherCAT电机适配器
 * 
 * 支持标准CiA 402协议的电机适配器基类
 */
class StandardMotorAdapter : public MotorAdapter {
public:
    MotorInfo getMotorInfo() const override;
    bool configurePdo(ec_slave_config_t* slave_config) override;
    std::vector<PdoConfig> getRxPdoConfig() const override;
    std::vector<PdoConfig> getTxPdoConfig() const override;
    MotorStatus readStatus(const uint8_t* domain_pd, const std::vector<unsigned int>& offset) const override;
    void writeControl(uint8_t* domain_pd, const std::vector<unsigned int>& offset, 
                     const MotorControl& control) const override;
    uint16_t generateControlWord(uint16_t current_status, bool target_enabled) const override;
    bool supportsMotor(uint32_t vendor_id, uint32_t product_code) const override;
    std::string getName() const override;
    uint16_t makeControl(uint16_t status, int32_t &start_pos, bool &run_enable) const override;

protected:
    /**
     * @brief 读取小端32位整数
     */
    static int32_t readInt32(const uint8_t* data);

    /**
     * @brief 读取小端16位整数
     */
    static int16_t readInt16(const uint8_t* data);

    /**
     * @brief 写入小端32位整数
     */
    static void writeInt32(uint8_t* data, int32_t value);

    /**
     * @brief 写入小端16位整数
     */
    static void writeInt16(uint8_t* data, int16_t value);
};

#endif // MOTOR_ADAPTER_HPP