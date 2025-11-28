#ifndef EU_ETHERCAT_H
#define EU_ETHERCAT_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef EXTERNFUNC
#ifdef _WIN32
#define EXTERNFUNC __declspec(dllexport)
#else
#define EXTERNFUNC
#endif
#endif

#define ETH_SUCCESS 0
#define ETH_FAILED_INIT 1
#define ETH_FAILED_NOSLAVE 2
#define ETH_FAILED_CHECKSTATE 3
#define ETH_FAILED_UNKNOWN 100

#ifdef _WIN32
    typedef char hint8;
    typedef short hint16;
    typedef long hint24;
    typedef long hint32;
    typedef long long hint40;
    typedef long long hint48;
    typedef long long hint56;
    typedef long long hint64;
    typedef unsigned char huint8;
    typedef unsigned short huint16;
    typedef unsigned long huint24;
    typedef unsigned long huint32;
    typedef unsigned long long huint40;
    typedef unsigned long long huint48;
    typedef unsigned long long huint56;
    typedef unsigned long long huint64;
    typedef float hreal32;
    typedef double hreal64;
#else
typedef signed char hint8;
typedef signed short int hint16;
typedef signed int hint24;
typedef signed int hint32;
typedef signed long int hint40;
typedef signed long int hint48;
typedef signed long int hint56;
typedef signed long int hint64;
typedef unsigned int huint8 __attribute__((__mode__(__QI__)));
typedef unsigned int huint16 __attribute__((__mode__(__HI__)));
typedef unsigned int huint24 __attribute__((__mode__(__SI__)));
typedef unsigned int huint32 __attribute__((__mode__(__SI__)));
typedef unsigned int huint40 __attribute__((__mode__(__DI__)));
typedef unsigned int huint48 __attribute__((__mode__(__DI__)));
typedef unsigned int huint56 __attribute__((__mode__(__DI__)));
typedef unsigned int huint64 __attribute__((__mode__(__DI__)));
typedef float hreal32;
typedef double hreal64;
#endif

    enum eth_DataType
    {
        //        eth_DataType_boolean = 0x01, /**< 布尔类型 */
        eth_DataType_int8 = 0x02,   /**< 1字节有符号整形 */
        eth_DataType_int16 = 0x03,  /**< 2字节有符号整形 */
        eth_DataType_int32 = 0x04,  /**< 4字节有符号整形 */
        eth_DataType_uint8 = 0x05,  /**< 1字节无符号整形 */
        eth_DataType_uint16 = 0x06, /**< 2字节无符号整形 */
        eth_DataType_uint32 = 0x07, /**< 4字节无符号整形 */
        eth_DataType_real32 = 0x08, /**< 4字节浮点型 */
        eth_DataType_real64 = 0x09  /**< 8字节浮点型 */
    };

    enum eth_State
    {
        eth_State_None = 0x00,
        eth_State_Init = 0x01,
        eth_State_Pre_OP = 0x02,
        eth_State_Boot = 0x03,
        eth_State_Safe_OP = 0x04,
        eth_State_Operational = 0x08,
        eth_State_ACK = 0x10,
        eth_State_ERROR = 0x10
    };

    enum eth_OperateMode
    {
        eth_OperateMode_AutoTuning = -4,
        eth_OperateMode_INLCalibration = -3,
        eth_OperateMode_RotorAligning = -2,
        eth_OperateMode_Reserve = 0,              /**< 保留的 */
        eth_OperateMode_ProfilePosition = 1,      /**< 轮廓位置模式 */
        eth_OperateMode_Velocity = 2,             /**< 速度模式 */
        eth_OperateMode_ProfileVelocity = 3,      /**< 轮廓速度模式 */
        eth_OperateMode_ProfileTorque = 4,        /**< 轮廓力矩模式 */
        eth_OperateMode_Homing = 6,               /**< 归航模式 */
        eth_OperateMode_InterpolatedPosition = 7, /**< 内插位置模式 */
        eth_OperateMode_CyclicSyncPosition = 8,   /**< 同步位置模式 */
        eth_OperateMode_CyclicSyncVelocity = 9,   /**< 同步速度模式 */
        eth_OperateMode_CyclicSyncTorque = 10,    /**< 同步力矩模式 */
        eth_OperateMode_TorquePositionFixed = 11  /**< 力矩位置混合模式 */
    };

    /**
     * @brief 打开ethercat接口，初始化主站，返回扫描到的从站数量
     *
     * @param ifName ethercat接口
     * @param ms
     * @param slaveCnt 存放扫描到的从站数量
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    ///
    EXTERNFUNC int eth_initDLL(const char *ifName, int ms, int *slaveCnt);

    /**
     * @brief 关闭设备，释放资源
     *
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_freeDLL();

    /**
     * @brief 获取从站状态
     *
     * @param slave 从站id
     * @param state 存放获取的从站状态
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_getSlaveState(huint16 slave, eth_State *state);

    /**
     * @brief 获取电机当前的操作模式
     *
     * @param slave 从站id
     * @param mode 存放读取的电机操作模式
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_getOperateMode(huint16 slave, eth_OperateMode *mode);

    /**
     * @brief 设置电机的操作模式
     *
     * @param slave 从站id
     * @param mode 操作模式
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setOperateMode(huint16 slave, eth_OperateMode mode);

    /**
     * @brief 设置电机的控制字
     *
     * @param slave 从站id
     * @param word 控制字
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setControlWord(huint16 slave, huint16 word);

    /**
     * @brief 获取电机当前状态字
     *
     * @param slave 从站id
     * @param word 存放读取的电机状态字
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_getStatusWord(huint16 slave, huint16 *word);

    /**
     * @brief 使能电机
     *
     * @param slave 从站id
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_enable(huint16 slave);

    /**
     * @brief 失能电机
     *
     * @param slave 从站id
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_disable(huint16 slave);

    /**
     * @brief 错误重置
     *
     * @param slave 从站id
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_faultReset(huint16 slave);

    /**
     * @brief 快速停机
     *
     * @param slave 从站id
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_quickStop(huint16 slave);

    /**
     * @brief 获取电机当前位置，单位脉冲
     *
     * @param slave 从站id
     * @param pos 存放获取到的位置
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_getActualPosition(huint16 slave, hint32 *pos);

    /**
     * @brief 获取电机当前速度，单位脉冲
     *
     * @param slave 从站id
     * @param vel 存放获取到的速度
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_getActualVelocity(huint16 slave, hint32 *vel);

    /**
     * @brief 获取电机当前力矩，单位千分之（额定力矩的千分之）
     *
     * @param slave 从站id
     * @param tor 存放获取到的力矩
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_getActualTorque(huint16 slave, hint16 *tor);

    /**
     * @brief 设置目标位置，单位脉冲
     *
     * @param slave 从站id
     * @param targetPos 目标位置
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setTargetPosition(huint16 slave, hint32 targetPos);

    /**
     * @brief 设置目标速度，单位脉冲
     *
     * @param slave 从站id
     * @param targetVel 目标速度
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setTargetVelocity(huint16 slave, hint32 targetVel);

    /**
     * @brief 设置轮廓速度，单位脉冲
     *
     * @param slave 从站id
     * @param profileVel 轮廓速度
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setProfileVelocity(huint16 slave, huint32 profileVel);

    /**
     * @brief 设置轮廓加速度，单位脉冲
     *
     * @param slave 从站id
     * @param profileAcc 轮廓加速度
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setProfileAcceleration(huint16 slave, huint32 profileAcc);

    /**
     * @brief 设置轮廓减速度，单位脉冲
     *
     * @param slave 从站id
     * @param profileDec 轮廓减速度
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setProfileDeceleration(huint16 slave, huint32 profileDec);

    /**
     * @brief 设置目标力矩，单位千分之
     *
     * @param slave 从站id
     * @param targetTor 目标力矩
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setTargetTorque(huint16 slave, hint32 targetTor);

    /**
     * @brief 设置力矩斜率,单位千分之/s²
     * 
     * @param slave 从站id
     * @param torSlope 增长斜率
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_setTorqueSlope(huint16 slave, huint32 torSlope);

    /**
     * @brief 读取从站字典
     *
     * @param slave 从站id
     * @param index 字典主索引
     * @param subIndex 字典子索引
     * @param value 存放读取的值
     * @param dataType 读取的数据类型
     * @param timeout 等待timeout ms来接收结果
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_readSDO(huint16 slave, huint16 index, huint8 subIndex, void *value, eth_DataType dataType, int timeout);

    /**
     * @brief 写从站字典
     *
     * @param slave 从站id
     * @param index 字典主索引
     * @param subIndex 字典子索引
     * @param value 写入值
     * @param dataType 写入的数据类型
     * @param timeout 等待timeout ms来确认结果
     * @return 成功返回ETH_SUCCESS，失败返回其他
     */
    EXTERNFUNC int eth_writeSDO(huint16 slave, huint16 index, huint8 subIndex, void *value, eth_DataType dataType, int timeout);

#ifdef __cplusplus
}
#endif

#endif