# 多厂商EtherCAT从站支持指南

## 概述

本系统现在支持通过解析 `ethercat slaves -v` 命令输出，自动检测和配置不同厂商的EtherCAT从站设备。

## 如何添加新的厂商支持

### 1. 获取新设备的VID和PID

首先运行以下命令获取设备的详细信息：
```bash
ethercat slaves -v
```

从输出中找到新设备的厂商ID（Vendor Id）和产品代码（Product code）：
```
Identity:
  Vendor Id:       0x00001097
  Product code:    0x00002406
```

### 2. 添加配置函数（如果需要）

在 `src/motor_api.cpp` 中添加新的配置函数：
```cpp
auto config_new_vendor_pdo = [](ec_slave_config_t* cfg) -> bool {
  printf("Configuring new vendor slave with custom PDO mapping\n");
  // 如果需要特殊的PDO配置，在这里添加
  return ecrt_slave_config_pdos(cfg, EC_END, slave_syncs) == 0;
};
```

### 3. 添加到支持列表

在 `supported_slaves` 数组中添加新设备：
```cpp
SupportedSlave supported_slaves[] = {
  {VENDOR_ID, PRODUCT_CODE, "EYOU Servo Motor", config_standard_pdo},
  {0x00001097, 0x00002406, "EYOU Servo Motor", config_standard_pdo},  // 现有设备
  {0x12345678, 0x87654321, "New Vendor Motor", config_new_vendor_pdo}, // 新设备
};
```

### 4. 重新编译

```bash
cd build
make
sudo ./test
```

## 示例：添加Delta伺服电机支持

假设你有一个Delta伺服电机，VID=0x000001DD，PID=0x12345678：

```cpp
// 1. 添加配置函数
auto config_delta_pdo = [](ec_slave_config_t* cfg) -> bool {
  printf("Configuring Delta servo motor\n");
  return ecrt_slave_config_pdos(cfg, EC_END, slave_syncs) == 0;
};

// 2. 添加到支持列表
SupportedSlave supported_slaves[] = {
  {VENDOR_ID, PRODUCT_CODE, "EYOU Servo Motor", config_standard_pdo},
  {0x000001DD, 0x12345678, "Delta Servo Motor", config_delta_pdo},
};
```

## 注意事项

1. **VID和PID必须准确**：确保从 `ethercat slaves -v` 输出中获取正确的值
2. **配置函数**：大多数设备可以使用标准配置函数，特殊设备可能需要自定义PDO映射
3. **测试**：添加新设备后，务必进行完整的功能测试
4. **错误处理**：系统会自动跳过不支持的设备，并在控制台输出相关信息

## 故障排除

如果新设备无法识别：

1. 检查VID和PID是否正确
2. 查看控制台输出，确认设备是否被检测到
3. 检查配置函数是否正确实现
4. 确保EtherCAT主站能够正常访问设备

## 支持的设备类型

当前支持：
- EYOU ServoModule_ECAT_V143 (VID: 0x00001097, PID: 0x00002406)

可以轻松扩展支持：
- Delta伺服电机
- Yaskawa伺服电机  
- Panasonic伺服电机
- 任何符合EtherCAT标准的伺服驱动器