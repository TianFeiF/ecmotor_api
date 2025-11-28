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
#include <stdbool.h>      // 提供bool类型定义
#include <netinet/in.h>   // 提供网络编程所需的结构体和常量
#include <sys/socket.h>   // 提供套接字API
#include <arpa/inet.h>    // 提供IP地址转换函数
#include <fcntl.h>        // 提供文件控制函数，用于设置非阻塞模式

/****************************************************************************/

#include "ecrt.h" // 引入EtherCAT主站的头文件，用于EtherCAT主站通信

/****************************************************************************/

#define TASK_FREQUENCY 4 * 1000 /* 任务执行频率，单位为 us */
#define WORK_MODE 8              /* 操作模式8：CSP（循环同步位置） */

/*****************************************************************************/

/* EtherCAT相关定义 */
static ec_master_t *master = NULL;          // 定义EtherCAT主站指针，初始化为NULL
static ec_master_state_t master_state = {}; // 定义EtherCAT主站状态结构体，初始化为空

static ec_domain_t *domain1 = NULL;          // 定义EtherCAT域指针，用于存储过程数据
static ec_domain_state_t domain1_state = {}; // 定义EtherCAT域状态结构体，初始化为空

static ec_slave_config_t *sc0 = NULL;
static ec_slave_config_t *sc1 = NULL;
static ec_slave_config_state_t sc0_state = {};
static ec_slave_config_state_t sc1_state = {};

/****************************************************************************/

/* Process Data */
static uint8_t *domain1_pd = NULL; // 定义指向域过程数据的指针

#define DM3E0 0, 0
#define DM3E1 0, 1
#define VID_PID 0x00001097, 0x00002406 /* 定义供应商ID和产品代码，确定设备类型 */

/* Offsets for PDO entries */
#pragma pack(push) // 保存当前对齐方式
#pragma pack(1)    // 将结构体按1字节对齐

// 存储PDO条目在过程数据中的偏移（ecrt注册后填充）
static struct {
    unsigned int controlWord;
    unsigned int workModeOut;
    unsigned int targetVelocity;
    unsigned int targetPosition;
    unsigned int PPcontourVel;
    unsigned int targetTorque;
} output[2];

static struct {
    unsigned int statusword;
    unsigned int workModeIn;
    unsigned int actualPosition;
    unsigned int actualVelocity;
    unsigned int actualTorque;
} input[2];

#pragma pack(pop) // 恢复之前保存的对齐方式

// 定义域中PDO条目的注册信息，描述需要注册的PDO条目
const static ec_pdo_entry_reg_t domain1_regs[] = {
    {DM3E0, VID_PID, 0x6040, 0, &output[0].controlWord, NULL},
    {DM3E0, VID_PID, 0x6060, 0, &output[0].workModeOut, NULL},
    {DM3E0, VID_PID, 0x60FF, 0, &output[0].targetVelocity, NULL},
    {DM3E0, VID_PID, 0x607A, 0, &output[0].targetPosition, NULL},
    {DM3E0, VID_PID, 0x60C2, 0, &output[0].PPcontourVel, NULL},
    {DM3E0, VID_PID, 0x6071, 0, &output[0].targetTorque, NULL},
    {DM3E0, VID_PID, 0x6041, 0, &input[0].statusword, NULL},
    {DM3E0, VID_PID, 0x6061, 0, &input[0].workModeIn, NULL},
    {DM3E0, VID_PID, 0x6064, 0, &input[0].actualPosition, NULL},
    {DM3E0, VID_PID, 0x606C, 0, &input[0].actualVelocity, NULL},
    {DM3E0, VID_PID, 0x6077, 0, &input[0].actualTorque, NULL},

    {DM3E1, VID_PID, 0x6040, 0, &output[1].controlWord, NULL},
    {DM3E1, VID_PID, 0x6060, 0, &output[1].workModeOut, NULL},
    {DM3E1, VID_PID, 0x60FF, 0, &output[1].targetVelocity, NULL},
    {DM3E1, VID_PID, 0x607A, 0, &output[1].targetPosition, NULL},
    {DM3E1, VID_PID, 0x60C2, 0, &output[1].PPcontourVel, NULL},
    {DM3E1, VID_PID, 0x6071, 0, &output[1].targetTorque, NULL},
    {DM3E1, VID_PID, 0x6041, 0, &input[1].statusword, NULL},
    {DM3E1, VID_PID, 0x6061, 0, &input[1].workModeIn, NULL},
    {DM3E1, VID_PID, 0x6064, 0, &input[1].actualPosition, NULL},
    {DM3E1, VID_PID, 0x606C, 0, &input[1].actualVelocity, NULL},
    {DM3E1, VID_PID, 0x6077, 0, &input[1].actualTorque, NULL},
    {}};

/***************************************************************************/
/* Config PDOs */
static ec_pdo_entry_info_t device_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x607a, 0x00, 32}, /* Target Position */
    {0x60ff, 0x00, 32}, /* Target Velocity */
    {0x6071, 0x00, 16}, /* Target Torque */
    {0x6060, 0x00, 8}, /* Modes of Operation */
    {0x60C2, 0x00, 8}, /* resv1 */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position Actual Value */
    {0x606c, 0x00, 32}, /* Velocity Actual Value */
    {0x6077, 0x00, 16}, /* Torque Actual Value */
    {0x6061, 0x00, 8}, /* Modes of Operation Display */
    {0x603f, 0x00, 16}, /* Error Code */
    {0x2026, 0x00, 8}, /* resv2 */
};

// 配置设备的PDO
static ec_pdo_info_t device_pdos[] = {
    {0x1600, 6, device_pdo_entries + 0}, /* RxPDO */
    {0x1a00, 7, device_pdo_entries + 6}, /* TxPDO */
};

// 定义同步管理器信息，用于PDO同步
static ec_sync_info_t device_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},            // 同步管理器0，输出，无PDO映射
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},             // 同步管理器1，输入，无PDO映射
    {2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_ENABLE},  // 与main.cpp一致：输出RxPDO启用看门狗
    {3, EC_DIR_INPUT, 1, device_pdos + 1, EC_WD_DISABLE},  // 输入TxPDO禁用看门狗
    {0xFF}};                                               // 结束标志，指示同步管理器配置结束

/**************************************************************************/

/*************************************************************************/

// TCP/HTTP 服务器与控制状态
#include <pthread.h>
#include <ctype.h>

static int http_port = 8080;                           // HTTP监听端口
static pthread_t http_thread;                          // HTTP服务器线程句柄
static pthread_mutex_t cmd_mutex = PTHREAD_MUTEX_INITIALIZER; // 保护命令的互斥锁

// 电机控制命令状态（对双电机同时生效）
typedef struct {
    bool run;   // 是否执行（运行/停止）
    int dir;    // 方向：1 正转，-1 反转，0 不动
    int step;   // 步长（每周期目标位置增量）
} motor_cmd_t;

static motor_cmd_t g_cmd = {false, 0, 0}; // 初始为未运行
static int32_t g_last_actual_pos[2] = {0, 0}; // 周期任务采样的实际位置

// 设置命令（带基本校验）
static void set_motor_cmd(bool run, int dir, int step)
{
    if (step < 1) step = 1;
    if (step > 100000) step = 100000; // 防止过大步长
    if (dir != -1 && dir != 0 && dir != 1) dir = 0;
    pthread_mutex_lock(&cmd_mutex);
    g_cmd.run = run;
    g_cmd.dir = dir;
    g_cmd.step = step;
    pthread_mutex_unlock(&cmd_mutex);
}

// 简易JSON解析：从形如 {"direction":"forward","step":123} 中提取字段
// 返回 0 成功，非0失败
static int parse_control_json(const char *body, int *out_dir, int *out_step)
{
    if (!body || !out_dir || !out_step) return -1;

    // 查找 direction
    const char *dkey = strstr(body, "\"direction\"");
    if (!dkey) return -2;
    const char *dcolon = strchr(dkey, ':');
    if (!dcolon) return -3;
    const char *dquote1 = strchr(dcolon, '"');
    if (!dquote1) return -4;
    const char *dquote2 = strchr(dquote1 + 1, '"');
    if (!dquote2) return -5;
    int dir = 0;
    size_t dlen = (size_t)(dquote2 - (dquote1 + 1));
    if (dlen > 32) return -6;
    char dval[40];
    memcpy(dval, dquote1 + 1, dlen);
    dval[dlen] = '\0';
    for (size_t i = 0; i < dlen; ++i) dval[i] = (char)tolower((unsigned char)dval[i]);
    if (strcmp(dval, "forward") == 0) dir = 1;
    else if (strcmp(dval, "reverse") == 0) dir = -1;
    else return -7;

    // 查找 step
    const char *skey = strstr(body, "\"step\"");
    if (!skey) return -8;
    const char *scolon = strchr(skey, ':');
    if (!scolon) return -9;
    long step = strtol(scolon + 1, NULL, 10);
    if (step <= 0 || step > 100000000) return -10; // 基本范围校验（解析级别）

    *out_dir = dir;
    *out_step = (int)step;
    return 0;
}

// 发送HTTP响应
static void http_send(int fd, const char *status, const char *ctype, const char *body)
{
    char header[512];
    int blen = body ? (int)strlen(body) : 0;
    int hlen = snprintf(header, sizeof(header),
                        "HTTP/1.1 %s\r\n"
                        "Content-Type: %s; charset=utf-8\r\n"
                        "Access-Control-Allow-Origin: *\r\n"
                        "Content-Length: %d\r\n"
                        "Connection: close\r\n\r\n",
                        status ? status : "200 OK",
                        ctype ? ctype : "text/plain",
                        blen);
    send(fd, header, hlen, 0);
    if (blen > 0) send(fd, body, blen, 0);
}

// 内嵌简单网页UI
static const char *UI_HTML =
"<!DOCTYPE html><html lang=\"zh\"><head><meta charset=\"utf-8\"><title>电机控制</title>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<style>body{font-family:sans-serif;margin:20px}label{display:inline-block;width:80px}button{margin:4px}input{margin:4px}</style>"
"</head><body>"
"<h3>TCP/HTTP 电机控制</h3>"
"<div><button id=\"btnConn\">连接</button><button id=\"btnDisc\">断开</button><span id=\"stat\">未连接</span></div>"
"<div><label>方向</label><select id=\"dir\"><option value=\"forward\">正转</option><option value=\"reverse\">反转</option></select></div>"
"<div><label>步长</label><input id=\"step\" type=\"number\" min=\"1\" value=\"5000\"></div>"
"<div><button id=\"btnExec\">执行</button><button id=\"btnStop\">停止</button></div>"
"<div><pre id=\"out\"></pre></div>"
"<script>"
"let connected=false;const out=document.getElementById('out');"
"function log(t){out.textContent=t}\n"
"document.getElementById('btnConn').onclick=()=>{connected=true;document.getElementById('stat').textContent='已连接'};"
"document.getElementById('btnDisc').onclick=()=>{connected=false;document.getElementById('stat').textContent='未连接'};"
"document.getElementById('btnExec').onclick=async()=>{if(!connected){log('未连接');return;}\n"
"const dir=document.getElementById('dir').value;const step=parseInt(document.getElementById('step').value||'0');\n"
"try{const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({direction:dir,step:step})});\n"
"const j=await r.text();log(j);}catch(e){log('错误:'+e)}};\n"
"document.getElementById('btnStop').onclick=async()=>{if(!connected){log('未连接');return;}\n"
"try{const r=await fetch('/stop',{method:'POST'});const t=await r.text();log(t);}catch(e){log('错误:'+e)}};\n"
"</script>"
"</body></html>";

// 处理HTTP请求（只支持简易GET/POST）
static void handle_http(int cfd, const char *req, int req_len)
{
    if (!req || req_len <= 0) {
        http_send(cfd, "400 Bad Request", "text/plain", "bad request");
        return;
    }
    // 方法与路径
    if (strncmp(req, "GET ", 4) == 0) {
        const char *path = req + 4;
        const char *sp = strchr(path, ' ');
        if (!sp) { http_send(cfd, "400 Bad Request", "text/plain", "bad request"); return; }
        size_t plen = (size_t)(sp - path);
        if (plen == 1 && path[0] == '/') {
            http_send(cfd, "200 OK", "text/html", UI_HTML);
        } else if (strncmp(path, "/status", plen) == 0) {
            char buf[256];
            pthread_mutex_lock(&cmd_mutex);
            int dir = g_cmd.dir; int step = g_cmd.step; bool run = g_cmd.run;
            pthread_mutex_unlock(&cmd_mutex);
            snprintf(buf, sizeof(buf), "{\"run\":%s,\"dir\":%d,\"step\":%d,\"pos0\":%d,\"pos1\":%d}",
                     run ? "true" : "false", dir, step, g_last_actual_pos[0], g_last_actual_pos[1]);
            http_send(cfd, "200 OK", "application/json", buf);
        } else {
            http_send(cfd, "404 Not Found", "text/plain", "not found");
        }
        return;
    }
    if (strncmp(req, "POST ", 5) == 0) {
        const char *path = req + 5;
        const char *sp = strchr(path, ' ');
        if (!sp) { http_send(cfd, "400 Bad Request", "text/plain", "bad request"); return; }
        size_t plen = (size_t)(sp - path);
        // 找到 body 起始
        const char *hdr_end = strstr(req, "\r\n\r\n");
        const char *body = hdr_end ? (hdr_end + 4) : NULL;
        if (strncmp(path, "/control", plen) == 0) {
            int dir = 0, step = 0; int rc = parse_control_json(body, &dir, &step);
            if (rc == 0) {
                set_motor_cmd(true, dir, step);
                http_send(cfd, "200 OK", "application/json", "{\"ok\":true}");
            } else {
                http_send(cfd, "400 Bad Request", "application/json", "{\"ok\":false,\"error\":\"invalid json\"}");
            }
        } else if (strncmp(path, "/stop", plen) == 0) {
            set_motor_cmd(false, 0, 0);
            http_send(cfd, "200 OK", "application/json", "{\"ok\":true}");
        } else {
            http_send(cfd, "404 Not Found", "text/plain", "not found");
        }
        return;
    }
    http_send(cfd, "405 Method Not Allowed", "text/plain", "method not allowed");
}

// HTTP服务器线程：监听并处理请求
static void *http_server_thread(void *arg)
{
    int port = (int)(intptr_t)arg;
    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
        perror("socket");
        return NULL;
    }
    int opt = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sfd);
        return NULL;
    }
    if (listen(sfd, 8) < 0) {
        perror("listen");
        close(sfd);
        return NULL;
    }
    printf("HTTP server listening on port %d\n", port);
    for (;;) {
        struct sockaddr_in cli; socklen_t cl = sizeof(cli);
        int cfd = accept(sfd, (struct sockaddr *)&cli, &cl);
        if (cfd < 0) { if (errno == EINTR) continue; perror("accept"); break; }
        char buf[4096];
        int n = recv(cfd, buf, sizeof(buf)-1, 0);
        if (n > 0) {
            buf[n] = '\0';
            handle_http(cfd, buf, n);
        }
        close(cfd);
    }
    close(sfd);
    return NULL;
}

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
    ec_slave_config_state_t s;       
    ecrt_slave_config_state(sc0, &s); 
    if (s.al_state != sc0_state.al_state) printf("slave0: State 0x%02X.\n", s.al_state);
    if (s.online != sc0_state.online) printf("slave0: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc0_state.operational) printf("slave0: %soperational.\n", s.operational ? "" : "Not ");
    sc0_state = s; 

    ecrt_slave_config_state(sc1, &s); 
    if (s.al_state != sc1_state.al_state) printf("slave1: State 0x%02X.\n", s.al_state);
    if (s.online != sc1_state.online) printf("slave1: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc1_state.operational) printf("slave1: %soperational.\n", s.operational ? "" : "Not ");
    sc1_state = s; 
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
    // 参考main.cpp的状态机逻辑
    static bool servo_enabled[2] = {false, false};
    static int32_t start_pos[2] = {0, 0};
    static int debug_counter = 0;
    
    // 读取当前状态
    uint16_t status = EC_READ_U16(domain1_pd + input[0].statusword);
    
    // 添加调试输出
    debug_counter++;
    if (debug_counter % 1000 == 0) {
        printf(" *********** Debug: status0=0x%04X, en0=%d, en1=%d ************\n", 
               EC_READ_U16(domain1_pd + input[0].statusword), 
               servo_enabled[0], servo_enabled[1]);
    }
    
    // 双电机状态机与CSP控制
    for (int i = 0; i < 2; ++i) {
        uint16_t status_i = EC_READ_U16(domain1_pd + input[i].statusword);
        uint16_t control_i = 0;
        if (!servo_enabled[i]) {
            uint8_t current_mode = EC_READ_S8(domain1_pd + input[i].workModeIn);
            printf("[M%d] Status: 0x%04X, Mode: %d\n", i, status_i, current_mode);
            switch (status_i & 0x6F) {
            case 0x00: // Not ready to switch on
                control_i = 0x06; // Shutdown
                break;
            case 0x40: // Switch on disabled
                control_i = 0x06; // Shutdown
                break;
            case 0x21: // Ready to switch on
                control_i = 0x07; // Switch on
                start_pos[i] = EC_READ_S32(domain1_pd + input[i].actualPosition);
                EC_WRITE_S32(domain1_pd + output[i].targetPosition, start_pos[i]);
                break;
            case 0x23: // Switched on
                control_i = 0x0F; // Enable operation
                break;
            case 0x27: // Operation enabled
                control_i = 0x0F; // Maintain
                servo_enabled[i] = true;
                break;
            default:
                control_i = 0x06; // Fallback
                break;
            }
            EC_WRITE_U16(domain1_pd + output[i].controlWord, control_i);
            EC_WRITE_S8(domain1_pd + output[i].workModeOut, WORK_MODE);
        } else {
            time_cnt++;
            if (time_cnt == 1) {
                start_pos[i] = EC_READ_S32(domain1_pd + input[i].actualPosition);
                EC_WRITE_S32(domain1_pd + output[i].targetPosition, start_pos[i]);
            } else {
                // 依据外部JSON命令控制目标位置增量
                pthread_mutex_lock(&cmd_mutex);
                int dir = g_cmd.dir; int step = g_cmd.step; bool run = g_cmd.run;
                pthread_mutex_unlock(&cmd_mutex);
                int delta = (run ? (dir * step) : 0);
                start_pos[i] += delta;
                EC_WRITE_S32(domain1_pd + output[i].targetPosition, start_pos[i]);
            }
            if (time_cnt % 1000 == 0) {
                int32_t actual_pos_i = EC_READ_S32(domain1_pd + input[i].actualPosition);
                printf("[M%d] targetPosition:%d actualPosition:%d\n", i,
                       EC_READ_S32(domain1_pd + output[i].targetPosition), actual_pos_i);
                g_last_actual_pos[i] = actual_pos_i;
            }
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
    if (!(sc0 = ecrt_master_slave_config(master, DM3E0, VID_PID))) // 配置从站0
    {
        fprintf(stderr, "Failed to get slave configuration for slave!\n");
        exit(EXIT_FAILURE); // 如果配置从站失败，退出程序
    }
    if (!(sc1 = ecrt_master_slave_config(master, DM3E1, VID_PID))) // 配置从站1
    {
        fprintf(stderr, "Failed to get slave configuration for slave1!\n");
        exit(EXIT_FAILURE);
    }
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc0, EC_END, device_syncs)) // 配置从站0的PDO映射
    {
        fprintf(stderr, "Failed to configure slave PDOs!\n");
        exit(EXIT_FAILURE); // 如果配置PDO失败，退出程序
    }
    else
    {
        printf("*Success to configuring slave PDOs*\n"); // 成功配置PDO
    }
    if (ecrt_slave_config_pdos(sc1, EC_END, device_syncs)) // 配置从站1的PDO映射
    {
        fprintf(stderr, "Failed to configure slave1 PDOs!\n");
        exit(EXIT_FAILURE);
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
    // 启动HTTP服务器线程，提供网页UI与JSON控制
    if (pthread_create(&http_thread, NULL, http_server_thread, (void *)(intptr_t)http_port) != 0) {
        perror("pthread_create");
    }

    while (1)
    {
        usleep(TASK_FREQUENCY); // 根据任务频率延时
        cyclic_task();          // 调用周期性任务函数
    }
    return EXIT_SUCCESS; // 正常结束程序
}
