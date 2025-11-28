/**
 * @file motor_api.cpp
 * @brief EtherCAT电机控制API实现文件
 * 
 * 实现了MotorApi类的所有功能，包括：
 * - EtherCAT主站和从站的初始化配置
 * - PDO条目的注册和管理
 * - 多电机的实时控制和状态监控
 * - 信号处理和资源清理
 */

#include <ecrt.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "motor_api.hpp"
#include "vendor_adapters.hpp"

/**
 * @brief ENI从站信息结构体
 * 存储从ENI文件解析的从站配置信息
 */
struct EniSlaveInfo {
    int position;           ///< 从站位置
    uint32_t vendor_id;     ///< 厂商ID
    uint32_t product_code;  ///< 产品代码
    uint32_t revision_no;   ///< 修订版本号
    uint32_t serial_no;     ///< 序列号
    std::string name;       ///< 设备名称
    bool has_dc;            ///< 是否支持分布式时钟
};



/**
 * @brief 简单的XML解析器类
 * 用于解析ENI XML文件提取从站信息
 */
class SimpleXmlParser {
private:
    std::string content_;
    size_t pos_;

    std::string extractValue(const std::string& tag) {
        std::string open_tag = "<" + tag + ">";
        std::string close_tag = "</" + tag + ">";
        
        size_t start = content_.find(open_tag, pos_);
        if (start == std::string::npos) return "";
        
        start += open_tag.length();
        size_t end = content_.find(close_tag, start);
        if (end == std::string::npos) return "";
        
        return content_.substr(start, end - start);
    }
    
    std::string extractAttribute(const std::string& tag, const std::string& attr) {
        size_t tag_start = content_.find("<" + tag, pos_);
        if (tag_start == std::string::npos) return "";
        
        std::string attr_pattern = attr + "=\"";
        size_t attr_start = content_.find(attr_pattern, tag_start);
        if (attr_start == std::string::npos) return "";
        
        attr_start += attr_pattern.length();
        size_t attr_end = content_.find("\"", attr_start);
        if (attr_end == std::string::npos) return "";
        
        return content_.substr(attr_start, attr_end - attr_start);
    }

public:
    SimpleXmlParser(const std::string& filename) : pos_(0) {
        std::ifstream file(filename);
        if (file.is_open()) {
            std::stringstream buffer;
            buffer << file.rdbuf();
            content_ = buffer.str();
            file.close();
        }
    }
    
    bool isValid() const {
        return !content_.empty() && 
               (content_.find("<EtherCATInfo") != std::string::npos || 
                content_.find("=== Master") != std::string::npos);
    }
    
    std::vector<EniSlaveInfo> parseSlaves() {
        std::vector<EniSlaveInfo> slaves;
        pos_ = 0;
        
        // 检查是文本格式还是XML格式
        if (content_.find("<EtherCATInfo") != std::string::npos) {
            return parseXmlSlaves();
        } else {
            return parseTextSlaves();
        }
    }
    
private:
    std::vector<EniSlaveInfo> parseTextSlaves() {
        std::vector<EniSlaveInfo> slaves;
        pos_ = 0;
        
        // 查找所有从站条目
        while (true) {
            size_t slave_start = content_.find("=== Master", pos_);
            if (slave_start == std::string::npos) break;
            
            pos_ = slave_start;
            EniSlaveInfo slave;
            
            // 解析从站位置
            size_t slave_pos_start = content_.find("Slave ", pos_);
            if (slave_pos_start != std::string::npos) {
                slave_pos_start += 6;
                size_t slave_pos_end = content_.find(" ===", slave_pos_start);
                if (slave_pos_end != std::string::npos) {
                    std::string pos_str = content_.substr(slave_pos_start, slave_pos_end - slave_pos_start);
                    slave.position = std::stoi(pos_str);
                }
            }
            
            // 解析身份信息 - 限制在当前从站段内搜索
            size_t next_slave_start = content_.find("=== Master", slave_start + 1);
            size_t search_end = (next_slave_start != std::string::npos) ? next_slave_start : content_.length();
            
            // 在当前从站段内查找Vendor Id
            size_t vendor_pos = content_.find("Vendor Id:", slave_start);
            if (vendor_pos != std::string::npos && vendor_pos < search_end) {
                size_t vendor_start = vendor_pos + 10; // 跳过 "Vendor Id:"
                while (vendor_start < content_.length() && (content_[vendor_start] == ' ' || content_[vendor_start] == '\t')) vendor_start++;
                size_t vendor_end = vendor_start;
                while (vendor_end < content_.length() && content_[vendor_end] != '\n' && content_[vendor_end] != '\r') vendor_end++;
                std::string vendor_str = content_.substr(vendor_start, vendor_end - vendor_start);
                if (!vendor_str.empty() && vendor_str.find("0x") == 0) {
                    slave.vendor_id = std::stoul(vendor_str.substr(2), nullptr, 16);
                }
            }
            
            // 在当前从站段内查找Product code
            size_t product_pos = content_.find("Product code:", slave_start);
            if (product_pos != std::string::npos && product_pos < search_end) {
                size_t product_start = product_pos + 13; // 跳过 "Product code:"
                while (product_start < content_.length() && (content_[product_start] == ' ' || content_[product_start] == '\t')) product_start++;
                size_t product_end = product_start;
                while (product_end < content_.length() && content_[product_end] != '\n' && content_[product_end] != '\r') product_end++;
                std::string product_str = content_.substr(product_start, product_end - product_start);
                if (!product_str.empty() && product_str.find("0x") == 0) {
                    slave.product_code = std::stoul(product_str.substr(2), nullptr, 16);
                }
            }
            
            std::string revision_str = extractValue("Revision number");
            if (!revision_str.empty() && revision_str.find("0x") == 0) {
                slave.revision_no = std::stoul(revision_str.substr(2), nullptr, 16);
            }
            
            std::string serial_str = extractValue("Serial number");
            if (!serial_str.empty() && serial_str.find("0x") == 0) {
                slave.serial_no = std::stoul(serial_str.substr(2), nullptr, 16);
            }
            
            // 解析设备名称
            size_t name_pos = content_.find("Device name:", slave_start);
            if (name_pos != std::string::npos && name_pos < search_end) {
                size_t name_start = name_pos + 12; // 跳过 "Device name:"
                while (name_start < content_.length() && (content_[name_start] == ' ' || content_[name_start] == '\t')) name_start++;
                size_t name_end = name_start;
                while (name_end < content_.length() && content_[name_end] != '\n' && content_[name_end] != '\r') name_end++;
                slave.name = content_.substr(name_start, name_end - name_start);
            }
            
            // 检查是否支持DC
            size_t dc_pos = content_.find("Distributed clocks: yes", slave_start);
            slave.has_dc = (dc_pos != std::string::npos && dc_pos < search_end);
            
            if (slave.vendor_id != 0 && slave.product_code != 0) {
                slaves.push_back(slave);
            }
            
            pos_ = slave_start + 1; // 继续查找下一个从站
        }
        
        return slaves;
    }
    
    std::vector<EniSlaveInfo> parseXmlSlaves() {
        std::vector<EniSlaveInfo> slaves;
        
        // 解析EtherCATInfo XML格式
        size_t pos = 0;
        
        // 查找所有Device条目
        while (true) {
            size_t device_start = content_.find("<Device>", pos);
            if (device_start == std::string::npos) break;
            
            size_t device_end = content_.find("</Device>", device_start);
            if (device_end == std::string::npos) break;
            
            EniSlaveInfo slave;
            
            // 解析Vendor ID
            size_t vid_start = content_.find("<VendorId>", device_start);
            if (vid_start != std::string::npos && vid_start < device_end) {
                vid_start += 10;
                size_t vid_end = content_.find("</VendorId>", vid_start);
                if (vid_end != std::string::npos && vid_end < device_end) {
                    std::string vid_str = content_.substr(vid_start, vid_end - vid_start);
                    slave.vendor_id = std::stoul(vid_str, nullptr, 16);
                }
            }
            
            // 解析Product Code
            size_t pid_start = content_.find("<ProductCode>", device_start);
            if (pid_start != std::string::npos && pid_start < device_end) {
                pid_start += 13;
                size_t pid_end = content_.find("</ProductCode>", pid_start);
                if (pid_end != std::string::npos && pid_end < device_end) {
                    std::string pid_str = content_.substr(pid_start, pid_end - pid_start);
                    slave.product_code = std::stoul(pid_str, nullptr, 16);
                }
            }
            
            // 解析设备名称
            size_t name_start = content_.find("<Name>", device_start);
            if (name_start != std::string::npos && name_start < device_end) {
                name_start += 6;
                size_t name_end = content_.find("</Name>", name_start);
                if (name_end != std::string::npos && name_end < device_end) {
                    slave.name = content_.substr(name_start, name_end - name_start);
                }
            }
            
            // 检查是否支持DC (通过DcSyncMode标签)
            size_t dc_start = content_.find("<DcSyncMode>", device_start);
            slave.has_dc = (dc_start != std::string::npos && dc_start < device_end);
            
            // 设置从站位置（XML格式通常不包含位置信息，使用顺序编号）
            slave.position = slaves.size(); // 从0开始
            
            if (slave.vendor_id != 0 && slave.product_code != 0) {
                slaves.push_back(slave);
            }
            
            pos = device_end + 9; // 继续查找下一个设备
        }
        
        return slaves;
    }
};

/**
 * @brief PDO条目索引枚举
 * 定义了EtherCAT从站中各个PDO条目的索引
 */
enum {
    CONTROL_WORD = 0,      ///< 控制字索引 (0x6040)
    TARGET_POS,              ///< 目标位置索引 (0x607A)
    TARGET_VELOCITY,         ///< 目标速度索引 (0x60FF)
    TARGET_TOR,              ///< 目标力矩索引 (0x6071)
    OP_MODE,                 ///< 操作模式索引 (0x6060)
    RESV1,                   ///< 保留参数1索引 (0x60C2)
    STATUS_WORD,             ///< 状态字索引 (0x6041)
    ACTUAL_POS,              ///< 实际位置索引 (0x6064)
    ACTUAL_VELOCITY,         ///< 实际速度索引 (0x606C)
    ACTUAN_TOR,              ///< 实际力矩索引 (0x6077)
    OP_MODE_DIS,             ///< 操作模式显示索引 (0x6061)
    RESV2,                   ///< 保留参数2索引 (0x2026)
    NUM_ENTRIES              ///< 条目总数
};

/**
 * @brief PDO条目信息定义
 * 定义了从站中各个PDO条目的COB-ID、子索引和位长度
 */
static ec_pdo_entry_info_t slave_pdo_entries[] = {
    {0x6040, 0x00, 16}, // Control word
    {0x607A, 0x00, 32}, // Target pos
    {0x60FF, 0x00, 32}, // Target velocity
    {0x6071, 0x00, 16}, // Target tor
    {0x6060, 0x00, 8},  // Operation mode
    {0x60C2, 0x00, 8},  // Reserved 1

    {0x6041, 0x00, 16}, // Status word
    {0x6064, 0x00, 32}, // Actual pos
    {0x606C, 0x00, 32}, // Actual velocity
    {0x6077, 0x00, 16}, // Actual tor
    {0x6061, 0x00, 8},  // Operation mode display
    {0x603F, 0x00, 16}, // Error code
    {0x2026, 0x00, 8},  // Reserved 2
};

/**
 * @brief RxPDO定义
 * 定义接收PDO映射，包含6个控制相关条目
 */
static ec_pdo_info_t slave_rx_pdo[] = {
    {0x1600, 6, slave_pdo_entries},
};

/**
 * @brief TxPDO定义
 * 定义发送PDO映射，包含7个状态相关条目
 */
static ec_pdo_info_t slave_tx_pdo[] = {
    {0x1A00, 7, &slave_pdo_entries[6]},
};

/**
 * @brief 同步管理器配置
 * 定义EtherCAT同步管理器的配置参数
 */
const static ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},    ///< SM0: 输出，无PDO，看门狗禁用
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},       ///< SM1: 输入，无PDO，看门狗禁用
    {2, EC_DIR_OUTPUT, 1, slave_rx_pdo, EC_WD_ENABLE}, ///< SM2: 输出，RxPDO，看门狗启用
    {3, EC_DIR_INPUT, 1, slave_tx_pdo, EC_WD_DISABLE}, ///< SM3: 输入，TxPDO，看门狗禁用
    {0xff}  ///< 结束标记
};

/**
 * @brief 小端32位整数读取函数
 * @param data 字节数据指针
 * @return 转换后的32位整数
 * 
 * 将从站的小端格式数据转换为本地32位整数
 */
int32_t read_le_int32(uint8_t *data)
{
  return static_cast<int32_t>(
    (data[0]) |
    (data[1] << 8) |
    (data[2] << 16) |
    (data[3] << 24));
}

/**
 * @brief 小端16位整数读取函数
 * @param data 字节数据指针
 * @return 转换后的16位整数
 * 
 * 将从站的小端格式数据转换为本地16位整数
 */
int16_t read_le_int16(const uint8_t *data) {
  return static_cast<int16_t>(
    (data[0]) |
    (data[1] << 8));
}

/**
 * @brief 全局活动API指针
 * 用于信号处理函数访问当前活动的MotorApi实例
 */
static MotorApi *g_active_api = nullptr;

/**
 * @brief 构造函数
 * 初始化所有成员变量为默认值，注册默认的电机适配器
 */
MotorApi::MotorApi()
  : master_(nullptr), domain_(nullptr), domain_pd_(nullptr), slave_count_(0), run_(true) {
  // 注册默认的电机适配器
  auto& manager = MotorAdapterManager::getInstance();
  manager.registerAdapter(std::make_shared<EyouMotorAdapter>());
  manager.registerAdapter(std::make_shared<DeltaMotorAdapter>());
  manager.registerAdapter(std::make_shared<YaskawaMotorAdapter>());
  manager.registerAdapter(std::make_shared<PanasonicMotorAdapter>());
}

/**
 * @brief 析构函数
 * 自动调用cleanup()释放资源
 */
MotorApi::~MotorApi() { cleanup(); }

/**
 * @brief 自动初始化EtherCAT系统
 * @return true 初始化成功，false 初始化失败
 * 
 * 该函数执行以下操作：
 * 1. 注册信号处理函数
 * 2. 请求EtherCAT主站
 * 3. 创建EtherCAT域
 * 4. 自动扫描并配置所有连接的从站
 * 5. 注册所有PDO条目
 * 6. 激活EtherCAT主站
 */
bool MotorApi::init_auto() {
  g_active_api = this;  // 设置全局活动API指针
  signal(SIGINT, MotorApi::signal_handler);  // 注册SIGINT信号处理
  
  // 请求EtherCAT主站
  master_ = ecrt_request_master(0);
  if (!master_) {
    printf("Failed to request EtherCAT master\n");
    return false;
  }
  
  // 创建EtherCAT域
  domain_ = ecrt_master_create_domain(master_);
  if (!domain_) {
    printf("Failed to create EtherCAT domain\n");
    return false;
  }

  // 自动扫描从站
  slave_count_ = 0;
  
  printf("Scanning for EtherCAT slaves...\n");
  
  // 首先通过系统命令获取实际的EtherCAT从站信息（详细模式）
  FILE* pipe = popen("ethercat slaves -v 2>/dev/null", "r");
  if (pipe) {
    char buffer[512];
    std::vector<std::pair<int, std::pair<uint32_t, uint32_t>>> slave_info; // position, (vendor_id, product_id)
    int current_slave = -1;
    uint32_t current_vid = 0;
    uint32_t current_pid = 0;
    
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      // 解析多行格式的输出
      int slave_num;
      
      // 检查是否是新的从站开始
      if (sscanf(buffer, "=== Master %*d, Slave %d ===", &slave_num) == 1) {
        current_slave = slave_num;
        printf("Parsing slave %d information...\n", current_slave);
      }
      
      // 解析厂商ID
      if (current_slave >= 0 && sscanf(buffer, "  Vendor Id: 0x%x", &current_vid) == 1) {
        printf("  Found Vendor ID: 0x%08X\n", current_vid);
      }
      
      // 解析产品代码
      if (current_slave >= 0 && sscanf(buffer, "  Product code: 0x%x", &current_pid) == 1) {
        printf("  Found Product code: 0x%08X\n", current_pid);
        
        // 当我们有完整的从站信息时，添加到列表
        if (current_vid != 0 && current_pid != 0) {
          slave_info.push_back({current_slave, {current_vid, current_pid}});
          printf("Added slave %d: VID=0x%08X, PID=0x%08X\n", current_slave, current_vid, current_pid);
          
          // 重置当前解析状态
          current_vid = 0;
          current_pid = 0;
        }
      }
    }
    pclose(pipe);
    
    printf("Total slaves parsed: %zu\n", slave_info.size());
    
    // 获取电机适配器管理器
    auto& adapter_manager = MotorAdapterManager::getInstance();
    
    // 为检测到的每个从站进行配置
    for (const auto& info : slave_info) {
      int pos = info.first;
      uint32_t vid = info.second.first;
      uint32_t pid = info.second.second;
      
      if (pos < 0 || pos > 31) continue; // 确保位置有效
      
      // 查找支持的电机适配器
      auto adapter = adapter_manager.findAdapter(vid, pid);
      if (adapter) {
        printf("Found adapter for slave at position %d (%s)\n", pos, adapter->getName().c_str());
        
        // 尝试配置从站
        ec_slave_config_t *cfg = ecrt_master_slave_config(master_, 0, pos, vid, pid);
        if (cfg) {
          // 使用适配器配置PDO
          if (adapter->configurePdo(cfg)) {
            scs_.push_back(cfg);
            slave_pos_.push_back(pos);
            motor_adapters_.push_back(adapter);
            ++slave_count_;
            printf("Successfully configured slave at position %d with adapter %s (total slaves: %zu)\n", 
                   pos, adapter->getName().c_str(), slave_count_);
          } else {
            printf("Failed to configure PDOs for slave at position %d\n", pos);
          }
        } else {
          printf("Failed to configure slave at position %d\n", pos);
        }
      } else {
        printf("Slave at position %d (VID=0x%08X, PID=0x%08X) has no supported adapter\n", 
               pos, vid, pid);
      }
    }
  } else {
    // 如果无法使用ethercat命令，回退到传统扫描方法
    printf("Could not execute 'ethercat slaves -v' command, falling back to manual scan...\n");
    printf("Warning: Manual scan may not support multi-vendor environments properly\n");
    
    // 扫描从站位置（最多16个）
    for (uint16_t pos = 0; pos < 16; ++pos) {
      // 尝试所有已注册的适配器
      auto& adapter_manager = MotorAdapterManager::getInstance();
      auto all_adapters = adapter_manager.getAllAdapters();
      
      for (const auto& adapter : all_adapters) {
        auto info = adapter->getMotorInfo();
        printf("Trying adapter %s for slave at position %d (VID=0x%08X, PID=0x%08X)\n", 
               adapter->getName().c_str(), pos, info.vendor_id, info.product_code);
        
        // 尝试使用适配器的VID/PID配置从站
        ec_slave_config_t *cfg = ecrt_master_slave_config(master_, 0, pos, 
                                                          info.vendor_id, 
                                                          info.product_code);
        if (cfg) {
          printf("Found potential slave at position %d with adapter %s\n", pos, adapter->getName().c_str());
          
          // 使用适配器配置PDO
          if (adapter->configurePdo(cfg)) {
            scs_.push_back(cfg);
            slave_pos_.push_back(pos);
            motor_adapters_.push_back(adapter);
            ++slave_count_;
            printf("Successfully configured slave at position %d with adapter %s (total slaves: %zu)\n", 
                   pos, adapter->getName().c_str(), slave_count_);
            break; // 找到适配器后跳出内层循环
          } else {
            printf("Failed to configure PDOs for slave at position %d with adapter %s\n", 
                   pos, adapter->getName().c_str());
          }
        }
      }
    }
  }
  
  if (slave_count_ == 0) {
    printf("No compatible motor slaves found\n");
    return false;
  }
  
  printf("Found %zu motor slaves\n", slave_count_);

  // 准备PDO条目注册数组
  regs_.clear();
  
  // 调整偏移量数组大小
  pdo_offsets_.resize(slave_count_);

  // 为每个从站注册PDO条目
  printf("=== PDO Registration Debug Info ===\n");
  
  // 首先尝试扫描从站的PDO配置
  printf("Scanning slave PDO configurations...\n");
  std::vector<ec_slave_config_t*> slave_configs(slave_count_, nullptr);
  
  for (size_t i = 0; i < slave_count_; ++i) {
    uint16_t pos = slave_pos_[i];
    auto adapter = motor_adapters_[i];
    auto motor_info = adapter->getMotorInfo();
    
    ec_slave_config_t *sc = ecrt_master_slave_config(master_, 0, pos, motor_info.vendor_id, motor_info.product_code);
    slave_configs[i] = sc;
    
    if (sc) {
      printf("Motor %zu: Found slave config for position %d (VID=0x%08X, PID=0x%08X)\n", 
             i, pos, motor_info.vendor_id, motor_info.product_code);
    } else {
      printf("Motor %zu: No slave config found for position %d (VID=0x%08X, PID=0x%08X)\n", 
             i, pos, motor_info.vendor_id, motor_info.product_code);
    }
  }
  
  for (size_t i = 0; i < slave_count_; ++i) {
    uint16_t pos = slave_pos_[i];
    auto adapter = motor_adapters_[i];
    auto motor_info = adapter->getMotorInfo();
    
    printf("Motor %zu: Position=%d, VID=0x%08X, PID=0x%08X, Adapter=%s\n", 
           i, pos, motor_info.vendor_id, motor_info.product_code, adapter->getName().c_str());
    
    // 使用之前获取的从站配置
    ec_slave_config_t *sc = slave_configs[i];
    if (!sc) {
      printf("  WARNING: No slave configuration found for position %d!\n", pos);
      continue;
    }
    
    // 配置从站的PDO映射
    printf("  Configuring PDO mappings...\n");
    if (!adapter->configurePdo(sc)) {
      printf("  ERROR: Failed to configure PDO mappings for position %d!\n", pos);
      continue;
    }
    printf("  PDO mappings configured successfully.\n");
    
    auto rx_pdo = adapter->getRxPdoConfig();
    auto tx_pdo = adapter->getTxPdoConfig();
    
    pdo_offsets_[i].resize(rx_pdo.size() + tx_pdo.size());
    
    printf("  RxPDO entries (%zu):\n", rx_pdo.size());
    for (size_t j = 0; j < rx_pdo.size(); ++j) {
      const auto& pdo = rx_pdo[j];
      printf("    [%zu] Index=0x%04X, SubIndex=0x%02X, OffsetPtr=%p\n", 
             j, pdo.index, pdo.subindex, &pdo_offsets_[i][j]);
      
      // 只注册有效的PDO条目（跳过gap fillers）
      if (pdo.index != 0x0000) {
        regs_.push_back({0, pos, motor_info.vendor_id, motor_info.product_code, 
                        pdo.index, pdo.subindex, &pdo_offsets_[i][j], NULL});
        printf("      -> Registering PDO entry\n");
      } else {
        printf("      -> Skipping gap filler\n");
      }
    }
    
    printf("  TxPDO entries (%zu):\n", tx_pdo.size());
    for (size_t j = 0; j < tx_pdo.size(); ++j) {
      const auto& pdo = tx_pdo[j];
      printf("    [%zu] Index=0x%04X, SubIndex=0x%02X, OffsetPtr=%p\n", 
             j, pdo.index, pdo.subindex, &pdo_offsets_[i][rx_pdo.size() + j]);
      
      // 只注册有效的PDO条目（跳过gap fillers）
      if (pdo.index != 0x0000) {
        regs_.push_back({0, pos, motor_info.vendor_id, motor_info.product_code, 
                        pdo.index, pdo.subindex, &pdo_offsets_[i][rx_pdo.size() + j], NULL});
        printf("      -> Registering PDO entry\n");
      } else {
        printf("      -> Skipping gap filler\n");
      }
    }
  }
  printf("=== End PDO Registration Debug ===\n");
  
  // 添加结束标记
  regs_.push_back({});
  
  // 注册所有PDO条目
  printf("Registering PDO entries with domain=%p, regs_data=%p\n", domain_, regs_.data());
  
  // 打印第一个注册条目的详细信息用于调试
  if (!regs_.empty() && regs_.size() > 0) {
    printf("First registration entry details:\n");
    printf("  alias=0x%04X, position=0x%04X, vendor_id=0x%08X, product_code=0x%08X\n",
           regs_[0].alias, regs_[0].position, regs_[0].vendor_id, regs_[0].product_code);
    printf("  index=0x%04X, subindex=0x%02X, offset=%p\n",
           regs_[0].index, regs_[0].subindex, regs_[0].offset);
  }
  
  int reg_result = ecrt_domain_reg_pdo_entry_list(domain_, regs_.data());
  if (reg_result != 0) {
    printf("Failed to register PDO entries - error code: %d\n", reg_result);
    printf("This usually means:\n");
    printf("  - PDO entry doesn't exist on the slave\n");
    printf("  - Vendor ID/Product code mismatch\n");
    printf("  - Slave is not in proper state\n");
    return false;
  }
  
  printf("PDO registration successful!\n");
  
  // 打印所有偏移值以验证注册结果
  printf("=== PDO Offset Results ===\n");
  for (size_t i = 0; i < slave_count_; ++i) {
    printf("Motor %zu PDO offsets:\n", i);
    auto rx_pdo = motor_adapters_[i]->getRxPdoConfig();
    auto tx_pdo = motor_adapters_[i]->getTxPdoConfig();
    
    printf("  RxPDO offsets: ");
    for (size_t j = 0; j < rx_pdo.size(); ++j) {
      printf("[%zu]=%u ", j, pdo_offsets_[i][j]);
    }
    printf("\n");
    
    printf("  TxPDO offsets: ");
    for (size_t j = 0; j < tx_pdo.size(); ++j) {
      printf("[%zu]=%u ", j, pdo_offsets_[i][rx_pdo.size() + j]);
    }
    printf("\n");
  }
  printf("=== End PDO Offset Results ===\n");
  printf("PDO entries registered successfully\n");
  
  // 打印所有分配的偏移量
  printf("=== PDO Offsets After Registration ===\n");
  for (size_t i = 0; i < slave_count_; ++i) {
    auto adapter = motor_adapters_[i];
    auto rx_pdo = adapter->getRxPdoConfig();
    auto tx_pdo = adapter->getTxPdoConfig();
    
    printf("Motor %zu: Total PDO entries = %zu\n", i, pdo_offsets_[i].size());
    printf("  RxPDO offsets:");
    for (size_t j = 0; j < rx_pdo.size(); ++j) {
      printf(" [%zu]=0x%04X:0x%02X->offset=%u", j, rx_pdo[j].index, rx_pdo[j].subindex, pdo_offsets_[i][j]);
    }
    printf("\n  TxPDO offsets:");
    for (size_t j = 0; j < tx_pdo.size(); ++j) {
      printf(" [%zu]=0x%04X:0x%02X->offset=%u", j, tx_pdo[j].index, tx_pdo[j].subindex, pdo_offsets_[i][rx_pdo.size() + j]);
    }
    printf("\n");
  }
  printf("=== End PDO Offsets ===\n");
  
  // 激活主站
  if (ecrt_master_activate(master_)) {
    printf("Failed to activate master\n");
    return false;
  }
  
  // 获取域数据指针
  domain_pd_ = ecrt_domain_data(domain_);
  if (!domain_pd_) {
    printf("Failed to get domain data\n");
    return false;
  }
  
  printf("Detected %zu motor slaves\n", slave_count_);
  
  if (slave_count_ == 0) {
    printf("Error: No motor slaves detected or configured successfully\n");
    return false;
  }
  
  printf("EtherCAT initialization completed successfully\n");
  return true;
}

/**
 * @brief 从ENI文件初始化EtherCAT网络
 * @param eni_filename ENI文件路径
 * @return true 初始化成功，false 初始化失败
 * 
 * 从EtherCAT Network Information (ENI) XML文件导入网络配置，
 * 包括从站信息、PDO映射等，然后激活EtherCAT主站。
 */
bool MotorApi::init_from_eni(const char* eni_filename) {
  printf("Initializing EtherCAT system from ENI file: %s\n", eni_filename);
  
  // 解析ENI文件
  SimpleXmlParser parser(eni_filename);
  if (!parser.isValid()) {
    printf("Error: Invalid ENI file or file not found: %s\n", eni_filename);
    return false;
  }
  
  std::vector<EniSlaveInfo> eni_slaves = parser.parseSlaves();
  if (eni_slaves.empty()) {
    printf("Error: No slaves found in ENI file\n");
    return false;
  }
  
  printf("Found %zu slaves in ENI file\n", eni_slaves.size());
  
  // 请求EtherCAT主站
  master_ = ecrt_request_master(0);
  if (!master_) {
    printf("Failed to request EtherCAT master\n");
    return false;
  }
  
  // 创建EtherCAT域
  domain_ = ecrt_master_create_domain(master_);
  if (!domain_) {
    printf("Failed to create EtherCAT domain\n");
    return false;
  }
  
  // 根据ENI文件配置从站
  slave_count_ = 0;
  
  printf("Configuring slaves from ENI file...\n");
  
  // 定义支持的从站类型列表（可以从ENI文件扩展）
  struct SupportedSlave {
    uint32_t vendor_id;
    uint32_t product_code;
    const char* name;
    bool (*config_func)(ec_slave_config_t* cfg);
  };
  
  auto config_standard_pdo = [](ec_slave_config_t* cfg) -> bool {
    return ecrt_slave_config_pdos(cfg, EC_END, slave_syncs) == 0;
  };
  
  // 获取电机适配器管理器
  auto& adapter_manager = MotorAdapterManager::getInstance();
  
  // 配置每个从站
  for (const auto& eni_slave : eni_slaves) {
    int pos = eni_slave.position;
    uint32_t vid = eni_slave.vendor_id;
    uint32_t pid = eni_slave.product_code;
    
    if (pos < 0 || pos > 31) {
      printf("Warning: Invalid slave position %d, skipping\n", pos);
      continue;
    }
    
    // 查找支持的电机适配器
    auto adapter = adapter_manager.findAdapter(vid, pid);
    if (adapter) {
      printf("Found adapter for ENI slave at position %d (%s)\n", pos, adapter->getName().c_str());
      
      // 尝试配置从站
      ec_slave_config_t *cfg = ecrt_master_slave_config(master_, 0, pos, vid, pid);
      if (cfg) {
        // 使用适配器配置PDO
        if (adapter->configurePdo(cfg)) {
          scs_.push_back(cfg);
          slave_pos_.push_back(pos);
          motor_adapters_.push_back(adapter);
          ++slave_count_;
          printf("Successfully configured ENI slave at position %d with adapter %s (total slaves: %zu)\n", 
                 pos, adapter->getName().c_str(), slave_count_);
        } else {
          printf("Failed to configure PDOs for ENI slave at position %d\n", pos);
        }
      } else {
        printf("Failed to configure ENI slave at position %d\n", pos);
      }
    } else {
      printf("ENI slave at position %d (VID=0x%08X, PID=0x%08X) has no supported adapter\n", 
             pos, vid, pid);
    }
  }
  
  if (slave_count_ == 0) {
    printf("No compatible ENI slaves found\n");
    return false;
  }
  
  printf("Found %zu ENI motor slaves\n", slave_count_);
  
  // 准备PDO条目注册数组（与init_auto相同）
  regs_.clear();
  
  // 调整偏移量数组大小
  pdo_offsets_.resize(slave_count_);

  // 为每个从站注册PDO条目
  for (size_t i = 0; i < slave_count_; ++i) {
    uint16_t pos = slave_pos_[i];
    auto adapter = motor_adapters_[i];
    auto motor_info = adapter->getMotorInfo();
    
    // 获取该电机的PDO配置
    auto rx_pdo = adapter->getRxPdoConfig();
    auto tx_pdo = adapter->getTxPdoConfig();
    
    // 为每个PDO条目分配偏移量
    pdo_offsets_[i].resize(rx_pdo.size() + tx_pdo.size());
    
    // 注册RxPDO条目（控制相关）
    for (size_t j = 0; j < rx_pdo.size(); ++j) {
      const auto& pdo = rx_pdo[j];
      regs_.push_back({0, pos, motor_info.vendor_id, motor_info.product_code, 
                      pdo.index, pdo.subindex, &pdo_offsets_[i][j], NULL});
    }
    
    // 注册TxPDO条目（状态相关）
    for (size_t j = 0; j < tx_pdo.size(); ++j) {
      const auto& pdo = tx_pdo[j];
      regs_.push_back({0, pos, motor_info.vendor_id, motor_info.product_code, 
                      pdo.index, pdo.subindex, &pdo_offsets_[i][rx_pdo.size() + j], NULL});
    }
  }
  
  // 添加结束标记
  regs_.push_back({});
  
  // 注册所有PDO条目
  if (ecrt_domain_reg_pdo_entry_list(domain_, regs_.data())) {
    printf("Failed to register PDO entries\n");
    return false;
  }
  
  // 激活主站
  if (ecrt_master_activate(master_)) {
    printf("Failed to activate master\n");
    return false;
  }
  
  // 获取域数据指针
  domain_pd_ = ecrt_domain_data(domain_);
  if (!domain_pd_) {
    printf("Failed to get domain data\n");
    return false;
  }
  
  printf("Detected %zu ENI motor slaves\n", slave_count_);
  
  printf("EtherCAT initialization from ENI file completed successfully\n");
  return true;
}

/**
 * @brief 接收并处理EtherCAT数据
 * 从主站接收数据并处理域数据
 */
void MotorApi::receive_and_process() {
  ecrt_master_receive(master_);
  ecrt_domain_process(domain_);
}

/**
 * @brief 排队并发送EtherCAT数据
 * 将域数据排队并发送到主站
 */
void MotorApi::queue_and_send() {
  ecrt_domain_queue(domain_);
  ecrt_master_send(master_);
}

/**
 * @brief 清理EtherCAT资源
 * 安全释放所有EtherCAT资源并重置状态
 */
void MotorApi::cleanup() {
  run_ = false;  // 设置运行标志为false
  
  // 释放主站资源
  if (master_) { 
    ecrt_release_master(master_); 
    master_ = nullptr; 
  }
  
  // 重置其他资源指针
  domain_ = nullptr;
  domain_pd_ = nullptr;
  
  // 清空所有数组
  for (auto *p : scs_) (void)p;  // 避免未使用警告
  scs_.clear();
  slave_count_ = 0;
  slave_pos_.clear();
  motor_adapters_.clear();
  pdo_offsets_.clear();
  regs_.clear();
}

std::string MotorApi::get_adapter_name(size_t motor) const {
  if (motor >= slave_count_) return "Invalid motor";
  return motor_adapters_[motor]->getName();
}

std::string MotorApi::get_motor_info(size_t motor) const {
  if (motor >= slave_count_) return "Invalid motor";
  
  auto info = motor_adapters_[motor]->getMotorInfo();
  char buffer[256];
  snprintf(buffer, sizeof(buffer), "VID: 0x%08X, PID: 0x%08X", 
           info.vendor_id, info.product_code);
  return std::string(buffer);
}

/**
 * @brief 信号处理函数
 * @param signum 信号编号
 * 
 * 处理SIGINT信号，触发清理操作
 */
void MotorApi::signal_handler(int) {
  if (g_active_api) {
    printf("\nReceived interrupt signal, cleaning up...\n");
    g_active_api->cleanup();
  }
}

/**
 * @brief 检查运行状态
 * @return true 运行中，false 已停止
 */
bool MotorApi::running() const { return run_; }

/**
 * @brief 获取电机数量
 * @return 检测到的电机数量
 */
size_t MotorApi::motor_count() const { return slave_count_; }

/**
 * @brief 设置操作模式
 * @param motor 电机索引
 * @param op_mode 操作模式
 * @param resv1_value 保留参数值
 * 
 * 将操作模式和保留参数写入指定电机的PDO
 */
void MotorApi::set_opmode(size_t motor, uint8_t op_mode, uint8_t resv1_value) {
  if (motor >= slave_count_) return;
  
  auto adapter = motor_adapters_[motor];
  auto rx_pdo = adapter->getRxPdoConfig();
  
  // 查找操作模式PDO条目
  for (size_t i = 0; i < rx_pdo.size(); ++i) {
    if (rx_pdo[i].index == 0x6060) {  // 操作模式
      memcpy(domain_pd_ + pdo_offsets_[motor][i], &op_mode, sizeof(uint8_t));
    }
    if (rx_pdo[i].index == 0x60C2) {  // 保留参数
      memcpy(domain_pd_ + pdo_offsets_[motor][i], &resv1_value, sizeof(uint8_t));
    }
  }
}

/**
 * @brief 获取状态字
 * @param motor 电机索引
 * @return 状态字值
 * 
 * 从指定电机的PDO中读取状态字
 */
uint16_t MotorApi::get_status(size_t motor) const {
  if (motor >= slave_count_) return 0;
  
  auto adapter = motor_adapters_[motor];
  auto tx_pdo = adapter->getTxPdoConfig();
  
  // 查找状态字PDO条目
  auto rx_pdo = adapter->getRxPdoConfig();
  for (size_t i = 0; i < tx_pdo.size(); ++i) {
    if (tx_pdo[i].index == 0x6041) {  // 状态字
      unsigned int offset = pdo_offsets_[motor][rx_pdo.size() + i];
      uint16_t status = *(uint16_t *)(domain_pd_ + offset);
      
      // 调试输出 - 只在Motor 0且状态变化时打印
      static uint16_t last_status[2] = {0, 0};
      if (motor == 0 && status != last_status[motor]) {
        printf("Motor %zu: Status read from offset %u = 0x%04X\n", motor, offset, status);
        last_status[motor] = status;
      }
      
      return status;
    }
  }
  return 0;
}

/**
 * @brief 生成控制字
 * @param motor 电机索引
 * @param status 当前状态字
 * @param start_pos 起始位置引用
 * @param run_enable 运行使能标志引用
 * @return 生成的控制字
 * 
 * 根据电机当前状态生成适当的控制字，管理状态机转换
 * 状态机转换逻辑：
 * - 0x00/0x40: 准备就绪，发送0x06进入准备状态
 * - 0x21: 准备状态，发送0x07使能操作，更新起始位置
 * - 0x23: 操作使能，发送0x0F进入运行状态
 * - 0x27: 运行状态，保持0x0F，设置运行使能标志
 */
uint16_t MotorApi::make_control(size_t motor, uint16_t status, int32_t &start_pos, bool &run_enable) {
  if (motor >= slave_count_) return 0;
  
  auto adapter = motor_adapters_[motor];
  return adapter->makeControl(status, start_pos, run_enable);
}

/**
 * @brief 写入控制字
 * @param motor 电机索引
 * @param control 控制字值
 * 
 * 将控制字写入指定电机的PDO
 */
void MotorApi::write_control(size_t motor, uint16_t control) {
  if (motor >= slave_count_) return;
  
  auto adapter = motor_adapters_[motor];
  auto rx_pdo = adapter->getRxPdoConfig();
  
  // 查找控制字PDO条目
  for (size_t i = 0; i < rx_pdo.size(); ++i) {
    if (rx_pdo[i].index == 0x6040) {  // 控制字
      *(uint16_t *)(domain_pd_ + pdo_offsets_[motor][i]) = control;
      break;
    }
  }
}

/**
 * @brief 更新目标位置
 * @param motor 电机索引
 * @param pos 目标位置值
 * 
 * 将目标位置写入指定电机的PDO
 */
void MotorApi::update_target_pos(size_t motor, int32_t pos) {
  if (motor >= slave_count_) return;
  
  auto adapter = motor_adapters_[motor];
  auto rx_pdo = adapter->getRxPdoConfig();
  
  // 查找目标位置PDO条目
  for (size_t i = 0; i < rx_pdo.size(); ++i) {
    if (rx_pdo[i].index == 0x607A) {  // 目标位置
      *(int32_t *)(domain_pd_ + pdo_offsets_[motor][i]) = pos;
      break;
    }
  }
}

int32_t MotorApi::get_actual_pos(size_t motor) const {
  if (motor >= slave_count_ || !domain_pd_) return 0;
  
  auto adapter = motor_adapters_[motor];
  auto tx_pdo = adapter->getTxPdoConfig();
  
  // 查找实际位置PDO条目
  auto rx_pdo = adapter->getRxPdoConfig();
  for (size_t i = 0; i < tx_pdo.size(); ++i) {
    if (tx_pdo[i].index == 0x6064) {  // 实际位置
      return read_le_int32(domain_pd_ + pdo_offsets_[motor][rx_pdo.size() + i]);
    }
  }
  return 0;
}

/**
 * @brief 复位电机
 * @param motor 电机索引
 * 
 * 发送复位命令（控制字0x0080）到指定电机
 * 控制字0x0080会触发电机驱动器的故障复位
 */
void MotorApi::reset(size_t motor) {
  if (motor >= slave_count_) return;
  
  auto adapter = motor_adapters_[motor];
  auto rx_pdo = adapter->getRxPdoConfig();
  
  // 查找控制字PDO条目
  for (size_t i = 0; i < rx_pdo.size(); ++i) {
    if (rx_pdo[i].index == 0x6040) {  // 控制字
      *(uint16_t *)(domain_pd_ + pdo_offsets_[motor][i]) = 0x0080;
      break;
    }
  }
}
