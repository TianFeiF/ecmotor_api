/**
 compile : gcc motor.c -o motor -I /usr/local/include -L /usr/local/lib -lethercat
 */

#include <errno.h>        // 提供错误码定义，用于处理错误
#include <signal.h>       // 提供信号处理函数，用于信号管理
#include <stdio.h>        // 提供输入输出函数，如 printf
#include <string.h>       // 提供字符串操作函数
#include <sys/resource.h> // 提供系统资源操作，如限制资源使用
#include <sys/time.h>     // 提供时间操作函数，用于计时操作
#include <sys/types.h>    // 提供基本系统数据类型定义
#include <unistd.h>       // 提供POSIX操作系统API，如睡眠函数
#include <sys/mman.h>     // 提供内存管理函数，如内存映射

/****************************************************************************/

#include "ecrt.h" // 引入EtherCAT主站的头文件，用于EtherCAT主站通信

/****************************************************************************/

#define TASK_FREQUENCY 4 * 1000 /* 任务执行频率，单位为 us */
#define WORK_MODE 8              /* 操作模式设置为3，对应0x6060:0的值 */

/*****************************************************************************/

/* EtherCAT相关定义 */
static ec_master_t *master = NULL;          // 定义EtherCAT主站指针，初始化为NULL
static ec_master_state_t master_state = {}; // 定义EtherCAT主站状态结构体，初始化为空

static ec_domain_t *domain1 = NULL;          // 定义EtherCAT域指针，用于存储过程数据
static ec_domain_state_t domain1_state = {}; // 定义EtherCAT域状态结构体，初始化为空

static ec_slave_config_t *sc = NULL;          // 定义EtherCAT从站配置指针
static ec_slave_config_state_t sc_state = {}; // 定义EtherCAT从站配置状态结构体，初始化为空

/****************************************************************************/

/* Process Data */
static uint8_t *domain1_pd = NULL; // 定义指向域过程数据的指针

#define DM3E 0, 0                      /* EtherCAT总线上的从站地址，使用从站地址0 */
#define VID_PID 0x000116c7, 0x003e0402 /* 定义供应商ID和产品代码，确定设备类型 */

/* Offsets for PDO entries */
#pragma pack(push) // 保存当前对齐方式
#pragma pack(1)    // 将结构体按1字节对齐

// 定义输出控制的PDO映射结构体
static struct
{
    uint16_t controlWord;        // 0x6040寄存器，控制字，用于控制驱动器的操作
    int8_t workModeOut;          // 0x6060寄存器，工作模式输出，用于选择操作模式
    int32_t targetPosition;      // 0x607A寄存器，目标位置，用于设定目标位置
    uint16_t touchProbeFunction; // 0x60B8寄存器，触摸探针功能
} output;                        // 输出控制结构体实例                  // 输出控制结构体实例

// 定义输入状态的PDO映射结构体
static struct
{
    uint16_t errorCode;          // 0x603F寄存器，错误代码
    uint16_t statusWord;         // 0x6041寄存器，状态字，用于指示驱动器状态
    int32_t actualPosition;      // 0x6064寄存器，实际位置，用于读取当前位置
    int8_t workModeIn;           // 0x6061寄存器，工作模式输入，用于显示当前操作模式
    uint16_t touchProbeStatus;   // 0x60B9寄存器，触摸探针状态
    int32_t touchProbePos1Value; // 0x60BA寄存器，触摸探针1位置值
    int32_t followingError;      // 0x60F4寄存器，跟随误差实际值
    uint32_t digitalInputs;      // 0x60FD寄存器，数字输入
    uint16_t servoErrorCode;     // 0x213F寄存器，伺服错误代码
} input;                         // 输入状态结构体实例

#pragma pack(pop) // 恢复之前保存的对齐方式

// 定义域中PDO条目的注册信息，描述需要注册的PDO条目
const static ec_pdo_entry_reg_t domain1_regs[] = {
    /**                 输出PDO (RxPDO) - 主站发送到从站               **/
    {DM3E, VID_PID, 0x6040, 0, &output.controlWord},        // 注册控制字偏移地址
    {DM3E, VID_PID, 0x6060, 0, &output.workModeOut},        // 注册操作模式偏移地址
    {DM3E, VID_PID, 0x607A, 0, &output.targetPosition},     // 注册目标位置偏移地址
    {DM3E, VID_PID, 0x60B8, 0, &output.touchProbeFunction}, // 注册触摸探针功能偏移地址
    
    /**                 输入PDO (TxPDO) - 从站发送到主站               **/
    {DM3E, VID_PID, 0x603F, 0, &input.errorCode},           // 注册错误代码偏移地址
    {DM3E, VID_PID, 0x6041, 0, &input.statusWord},          // 注册状态字偏移地址
    {DM3E, VID_PID, 0x6064, 0, &input.actualPosition},      // 注册实际位置偏移地址
    {DM3E, VID_PID, 0x6061, 0, &input.workModeIn},          // 注册工作模式输入偏移地址
    {DM3E, VID_PID, 0x60B9, 0, &input.touchProbeStatus},    // 注册触摸探针状态偏移地址
    {DM3E, VID_PID, 0x60BA, 0, &input.touchProbePos1Value}, // 注册触摸探针位置值偏移地址
    {DM3E, VID_PID, 0x60F4, 0, &input.followingError},      // 注册跟随误差偏移地址
    {DM3E, VID_PID, 0x60FD, 0, &input.digitalInputs},       // 注册数字输入偏移地址
    {DM3E, VID_PID, 0x213F, 0, &input.servoErrorCode},      // 注册伺服错误代码偏移地址
    
    {}}; // 结束符，指示PDO条目注册结束

/***************************************************************************/
/* Config PDOs */
// static ec_pdo_entry_info_t device_pdo_entries[] = {
//     /* RxPdo 0x1600：接收PDO定义 */
//     {0x6040, 0x00, 16}, // 控制字，16位宽度
//     {0x6060, 0x00, 8},  // 操作模式，8位宽度
//     {0x60FF, 0x00, 32}, // 目标速度，32位宽度
//     {0x607A, 0x00, 32}, // 目标位置，32位宽度
//     {0x6081, 0x00, 32}, // 梯形速度，32位宽度
//     {0x6071, 0x00, 16}, // 目标力矩，16位宽度
//     /* TxPdo 0x1A00：发送PDO定义 */
//     {0x6041, 0x00, 16}, // 状态字，16位宽度
//     {0x6061, 0x00, 8},  // 工作模式输入，8位宽度
//     {0x6064, 0x00, 32}, // 实际位置，32位宽度
//     {0x606C, 0x00, 32}, // 实际速度，32位宽度
//     {0x6077, 0x00, 16}  // 实际力矩，16位宽度
// };

static ec_pdo_entry_info_t device_pdo_entries[] = {
    /* RxPdo 0x1600：接收PDO定义（主站→从站） */
    {0x6040, 0x00, 16}, // 控制字，16位宽度
    {0x6060, 0x00, 8},  // 操作模式，8位宽度
    {0x607A, 0x00, 32}, // 目标位置，32位宽度
    {0x60B8, 0x00, 16}, // 触摸探针功能，16位宽度
    
    /* TxPdo 0x1A00：发送PDO定义（从站→主站） */
    {0x603F, 0x00, 16}, // 错误代码，16位宽度
    {0x6041, 0x00, 16}, // 状态字，16位宽度
    {0x6064, 0x00, 32}, // 实际位置，32位宽度
    {0x6061, 0x00, 8},  // 工作模式显示，8位宽度
    {0x60B9, 0x00, 16}, // 触摸探针状态，16位宽度
    {0x60BA, 0x00, 32}, // 触摸探针位置值，32位宽度
    {0x60F4, 0x00, 32}, // 跟随误差实际值，32位宽度
    {0x60FD, 0x00, 32}, // 数字输入，32位宽度
    {0x213F, 0x00, 16}  // 伺服错误代码，16位宽度
};
// 配置设备的PDO
static ec_pdo_info_t device_pdos[] = {
    // 配置发送PDO (TxPDO)
    {0x1600, 4, device_pdo_entries + 0}, // 发送PDO包含6个条目
    // 配置接收PDO (RxPDO)
    {0x1A00, 9, device_pdo_entries + 4}}; // 接收PDO包含5个条目

// 定义同步管理器信息，用于PDO同步
static ec_sync_info_t device_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},            // 同步管理器0，输出，无PDO映射
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},             // 同步管理器1，输入，无PDO映射
    {2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_ENABLE}, // 同步管理器2，输出，包含1个RxPDO
    {3, EC_DIR_INPUT, 1, device_pdos + 1, EC_WD_DISABLE},  // 同步管理器3，输入，包含1个TxPDO
    {0xFF}};                                               // 结束标志，指示同步管理器配置结束

/**************************************************************************/

/*************************************************************************/

// 检查域1的状态
void check_domain1_state(void)
{
    ec_domain_state_t ds;            // 定义域状态结构体
    ecrt_domain_state(domain1, &ds); // 获取当前域的状态
    if (ds.working_counter != domain1_state.working_counter)
    {
        printf("Domain1: WC %u.\n", ds.working_counter); // 打印工作计数器
    }
    if (ds.wc_state != domain1_state.wc_state)
    {
        printf("Domain1: State %u.\n", ds.wc_state); // 打印工作计数器状态
    }
    domain1_state = ds; // 更新当前域状态
}

// 检查主站的状态
void check_master_state(void)
{
    ec_master_state_t ms;           // 定义主站状态结构体
    ecrt_master_state(master, &ms); // 获取当前主站的状态
    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding); // 打印响应的从站数量
    }
    if (ms.al_states != master_state.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states); // 打印应用层状态
    }
    if (ms.link_up != master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down"); // 打印链路状态
    }
    master_state = ms; // 更新当前主站状态
}

/****************************************************************************/

// 检查从站配置状态
void check_slave_config_states(void)
{
    ec_slave_config_state_t s;       // 定义从站配置状态结构体
    ecrt_slave_config_state(sc, &s); // 获取从站配置状态
    if (s.al_state != sc_state.al_state)
    {
        printf("slave: State 0x%02X.\n", s.al_state); // 打印从站应用层状态
    }
    if (s.online != sc_state.online)
    {
        printf("slave: %s.\n", s.online ? "online" : "offline"); // 打印从站在线状态
    }
    if (s.operational != sc_state.operational)
    {
        printf("slave: %soperational.\n", s.operational ? "" : "Not "); // 打印从站操作状态
    }
    sc_state = s; // 更新当前从站配置状态
}

/*******************************************************************************/

// 周期性任务函数，用于控制伺服的状态
void cyclic_task()
{
    static uint16_t flagstate = 0; // 用于跟踪伺服状态的标志变量
    static uint32_t time_cnt = 0;
    static int32_t outpos = 0, inpos = 0;

    ecrt_master_receive(master);  // 接收过程数据
    ecrt_domain_process(domain1); // 处理域中的过程数据

    check_domain1_state(); // 检查域的状态

    check_master_state();        // 检查主站状态
    check_slave_config_states(); // 检查从站配置状态
    // 使能伺服控制逻辑
    if (flagstate <= 1200)
    {

        flagstate++; // 增加flagstate以控制状态转换

        switch (flagstate)
        {
        case 1:

            EC_WRITE_U16(domain1_pd + output.controlWord, 0x80); // 重置驱动器错误
            break;
        case 500:

            EC_WRITE_U16(domain1_pd + output.controlWord, 0x0006);   // 进入准备使能状态
            EC_WRITE_S8(domain1_pd + output.workModeOut, WORK_MODE); // 设置操作模式为速度模式
            break;
        case 600:
            EC_WRITE_U16(domain1_pd + output.controlWord, 0x0007); // 使能伺服驱动器
            break;
        case 800:
            EC_WRITE_U16(domain1_pd + output.controlWord, 0x000F); // 完全使能伺服
            outpos = EC_READ_S32(domain1_pd + input.actualPosition);
	    inpos = outpos;
            printf(" *********** actualPosition : %d   ************  \n ", outpos);
            break;
        }
    }
    else
    {
	inpos = EC_READ_S32(domain1_pd + input.actualPosition);
        time_cnt++;
        if (time_cnt < 1000)
        {
            outpos = inpos + time_cnt * 100;
        }
        else
        {
            static flagOK = 1;
            if (flagOK == 1)
            {
                printf("\n\n ******************* flagOK AAAA add ok ! ********************  outpos : %d \n\n", outpos);
                flagOK = 2;
            }
            outpos = inpos + 100000;
        }
        EC_WRITE_S32(domain1_pd + output.targetPosition, outpos);

        if (time_cnt % 1000 == 0)
        {
            
            printf(" *********** targetPosition  : %d   ,   actualPosition : %d ********\n ", outpos, inpos);
        }
    }

    ecrt_domain_queue(domain1); // 将数据排入发送队列
    ecrt_master_send(master);   // 发送过程数据
}

/****************************************************************************/

int main(int argc, char **argv)
{
    printf("\n\r ******* ChangZhuKeZhan  CSP  mode  2024  !  *******  \n\n");
    printf("Requesting master...\n");
    master = ecrt_request_master(0); // 请求EtherCAT主站实例
    if (!master)
    {
        exit(EXIT_FAILURE); // 如果请求失败，退出程序
    }

    domain1 = ecrt_master_create_domain(master); // 创建EtherCAT域，管理过程数据
    if (!domain1)
    {
        exit(EXIT_FAILURE); // 如果创建域失败，退出程序
    }
    if (!(sc = ecrt_master_slave_config(master, DM3E, VID_PID))) // 配置EtherCAT从站
    {
        fprintf(stderr, "Failed to get slave configuration for slave!\n");
        exit(EXIT_FAILURE); // 如果配置从站失败，退出程序
    }
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc, EC_END, device_syncs)) // 配置从站的PDO映射
    {
        fprintf(stderr, "Failed to configure slave PDOs!\n");
        exit(EXIT_FAILURE); // 如果配置PDO失败，退出程序
    }
    else
    {
        printf("*Success to configuring slave PDOs*\n"); // 成功配置PDO
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) // 注册域中的PDO条目
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        exit(EXIT_FAILURE); // 如果注册PDO条目失败，退出程序
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) // 激活EtherCAT主站
    {
        exit(EXIT_FAILURE); // 如果激活失败，退出程序
    }
    else
    {
        printf("*Master activated*\n"); // 成功激活主站
    }
    if (!(domain1_pd = ecrt_domain_data(domain1))) // 获取域数据指针
    {
        exit(EXIT_FAILURE); // 如果获取数据指针失败，退出程序
    }

    printf("*It's working now*\n");

    while (1)
    {
        usleep(TASK_FREQUENCY); // 根据任务频率延时
        cyclic_task();          // 调用周期性任务函数
    }
    return EXIT_SUCCESS; // 正常结束程序
}

