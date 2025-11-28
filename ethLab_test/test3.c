/*
 * EtherCAT 三从站 CSP 控制示例程序
 * 提供：HTTP 控制界面与 JSON 接口、PDO 状态诊断输出、三从站状态机与 CSP 位置控制
 * 适用：EtherLab (ecrt) 主站，CiA-402 伺服驱动
 */
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctype.h>
#include "ecrt.h"

/* 控制循环周期（微秒）。与伺服插值周期匹配可获得更平滑控制 */
#define TASK_FREQUENCY (4 * 1000)
#define DC_SYNC0_PERIOD_NS (TASK_FREQUENCY * 1000)
/* 操作模式：8 = CSP（循环同步位置） */
#define WORK_MODE 8
/* 每周期允许的最大目标增量，限制跟随误差峰值 */
#define MAX_DELTA_PER_CYCLE 400000

/* EtherCAT 主站/域/从站配置状态 */
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};
static ec_slave_config_t *sc0 = NULL;
static ec_slave_config_t *sc1 = NULL;
static ec_slave_config_t *sc2 = NULL;
static ec_slave_config_state_t sc0_state = {};
static ec_slave_config_state_t sc1_state = {};
static ec_slave_config_state_t sc2_state = {};
static uint8_t *domain1_pd = NULL;

#define SLV0 0, 0  /* 从站0位置（bus pos） */
#define SLV1 0, 1  /* 从站1位置（bus pos） */
#define SLV2 0, 2  /* 从站2位置（bus pos） */
#define VENDOR_PRODUCT 0x000116c7, 0x003e0402  /* 供应商ID、产品码 */

#pragma pack(push)
#pragma pack(1)  /* 按1字节对齐，确保 PDO 偏移与从站一致 */
/* 输出 PDO 偏移集合：主站 → 从站 */
static struct {
    unsigned int controlWord;
    unsigned int workModeOut;
    unsigned int targetPosition;
    unsigned int touchProbeFunc;
} output[3];

/* 输入 PDO 偏移集合：从站 → 主站 */
static struct {
    unsigned int statusword;
    unsigned int workModeIn;
    unsigned int actualPosition;
    unsigned int errorCode;
    unsigned int followingError;
    unsigned int digitalInputs;
    unsigned int touchProbeStatus;
    unsigned int touchProbePos;
    unsigned int servoErrorCode;
} input[3];
#pragma pack(pop)

/* PDO 条目在域中的注册，建立变量与 PDO 映射关系 */
const static ec_pdo_entry_reg_t domain1_regs[] = {
    {SLV0, VENDOR_PRODUCT, 0x6040, 0, &output[0].controlWord, NULL},
    {SLV0, VENDOR_PRODUCT, 0x6060, 0, &output[0].workModeOut, NULL},
    {SLV0, VENDOR_PRODUCT, 0x607A, 0, &output[0].targetPosition, NULL},
    {SLV0, VENDOR_PRODUCT, 0x60B8, 0, &output[0].touchProbeFunc, NULL},
    {SLV0, VENDOR_PRODUCT, 0x6041, 0, &input[0].statusword, NULL},
    {SLV0, VENDOR_PRODUCT, 0x6064, 0, &input[0].actualPosition, NULL},
    {SLV0, VENDOR_PRODUCT, 0x6061, 0, &input[0].workModeIn, NULL},
    {SLV0, VENDOR_PRODUCT, 0x603F, 0, &input[0].errorCode, NULL},
    {SLV0, VENDOR_PRODUCT, 0x60F4, 0, &input[0].followingError, NULL},
    {SLV0, VENDOR_PRODUCT, 0x60FD, 0, &input[0].digitalInputs, NULL},
    {SLV0, VENDOR_PRODUCT, 0x60B9, 0, &input[0].touchProbeStatus, NULL},
    {SLV0, VENDOR_PRODUCT, 0x60BA, 0, &input[0].touchProbePos, NULL},
    {SLV0, VENDOR_PRODUCT, 0x213F, 0, &input[0].servoErrorCode, NULL},

    {SLV1, VENDOR_PRODUCT, 0x6040, 0, &output[1].controlWord, NULL},
    {SLV1, VENDOR_PRODUCT, 0x6060, 0, &output[1].workModeOut, NULL},
    {SLV1, VENDOR_PRODUCT, 0x607A, 0, &output[1].targetPosition, NULL},
    {SLV1, VENDOR_PRODUCT, 0x60B8, 0, &output[1].touchProbeFunc, NULL},
    {SLV1, VENDOR_PRODUCT, 0x6041, 0, &input[1].statusword, NULL},
    {SLV1, VENDOR_PRODUCT, 0x6064, 0, &input[1].actualPosition, NULL},
    {SLV1, VENDOR_PRODUCT, 0x6061, 0, &input[1].workModeIn, NULL},
    {SLV1, VENDOR_PRODUCT, 0x603F, 0, &input[1].errorCode, NULL},
    {SLV1, VENDOR_PRODUCT, 0x60F4, 0, &input[1].followingError, NULL},
    {SLV1, VENDOR_PRODUCT, 0x60FD, 0, &input[1].digitalInputs, NULL},
    {SLV1, VENDOR_PRODUCT, 0x60B9, 0, &input[1].touchProbeStatus, NULL},
    {SLV1, VENDOR_PRODUCT, 0x60BA, 0, &input[1].touchProbePos, NULL},
    {SLV1, VENDOR_PRODUCT, 0x213F, 0, &input[1].servoErrorCode, NULL},

    {SLV2, VENDOR_PRODUCT, 0x6040, 0, &output[2].controlWord, NULL},
    {SLV2, VENDOR_PRODUCT, 0x6060, 0, &output[2].workModeOut, NULL},
    {SLV2, VENDOR_PRODUCT, 0x607A, 0, &output[2].targetPosition, NULL},
    {SLV2, VENDOR_PRODUCT, 0x60B8, 0, &output[2].touchProbeFunc, NULL},
    {SLV2, VENDOR_PRODUCT, 0x6041, 0, &input[2].statusword, NULL},
    {SLV2, VENDOR_PRODUCT, 0x6064, 0, &input[2].actualPosition, NULL},
    {SLV2, VENDOR_PRODUCT, 0x6061, 0, &input[2].workModeIn, NULL},
    {SLV2, VENDOR_PRODUCT, 0x603F, 0, &input[2].errorCode, NULL},
    {SLV2, VENDOR_PRODUCT, 0x60F4, 0, &input[2].followingError, NULL},
    {SLV2, VENDOR_PRODUCT, 0x60FD, 0, &input[2].digitalInputs, NULL},
    {SLV2, VENDOR_PRODUCT, 0x60B9, 0, &input[2].touchProbeStatus, NULL},
    {SLV2, VENDOR_PRODUCT, 0x60BA, 0, &input[2].touchProbePos, NULL},
    {SLV2, VENDOR_PRODUCT, 0x213F, 0, &input[2].servoErrorCode, NULL},
    {}
};

/* 设备的 PDO 条目描述（用于 ecrt_slave_config_pdos） */
static ec_pdo_entry_info_t device_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x607a, 0x00, 32},
    {0x60b8, 0x00, 16},
    {0x603f, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x6061, 0x00, 8},
    {0x60b9, 0x00, 16},
    {0x60ba, 0x00, 32},
    {0x60f4, 0x00, 32},
    {0x60fd, 0x00, 32},
    {0x213f, 0x00, 16},
};

/* PDO 映射：RxPDO / TxPDO */
static ec_pdo_info_t device_pdos[] = {
    {0x1600, 4, device_pdo_entries + 0},
    {0x1a00, 9, device_pdo_entries + 4},
};

/* 同步管理器配置：2=RxPDO，3=TxPDO */
static ec_sync_info_t device_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, device_pdos + 1, EC_WD_DISABLE},
    {0xFF}
};

/* HTTP 服务配置与命令缓冲 */
static int http_port = 8080;
static pthread_t http_thread;
static pthread_mutex_t cmd_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 简单的运动命令：是否运行、方向、步长 */
typedef struct { bool run; int dir; int step; } motor_cmd_t;
static motor_cmd_t g_cmd = {false, 0, 0};
static int32_t g_last_actual_pos[3] = {0, 0, 0};
static volatile sig_atomic_t g_stop = 0;

static uint64_t monotonic_ns(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}
static void sig_handler(int sig) { (void)sig; g_stop = 1; }

/* 原子更新运动命令（带范围检查） */
static void set_motor_cmd(bool run, int dir, int step) {
    if (step < 1) step = 1;
    if (step > 100000) step = 100000;
    if (dir != -1 && dir != 0 && dir != 1) dir = 0;
    pthread_mutex_lock(&cmd_mutex);
    g_cmd.run = run; g_cmd.dir = dir; g_cmd.step = step;
    pthread_mutex_unlock(&cmd_mutex);
}

/* 轻量 JSON 解析：提取 {direction, step} */
static int parse_control_json(const char *body, int *out_dir, int *out_step) {
    if (!body || !out_dir || !out_step) return -1;
    const char *dkey = strstr(body, "\"direction\""); if (!dkey) return -2;
    const char *dcolon = strchr(dkey, ':'); if (!dcolon) return -3;
    const char *dquote1 = strchr(dcolon, '"'); if (!dquote1) return -4;
    const char *dquote2 = strchr(dquote1 + 1, '"'); if (!dquote2) return -5;
    int dir = 0; size_t dlen = (size_t)(dquote2 - (dquote1 + 1)); if (dlen > 32) return -6;
    char dval[40]; memcpy(dval, dquote1 + 1, dlen); dval[dlen] = '\0';
    for (size_t i = 0; i < dlen; ++i) dval[i] = (char)tolower((unsigned char)dval[i]);
    if (strcmp(dval, "forward") == 0) dir = 1; else if (strcmp(dval, "reverse") == 0) dir = -1; else return -7;
    const char *skey = strstr(body, "\"step\""); if (!skey) return -8;
    const char *scolon = strchr(skey, ':'); if (!scolon) return -9;
    long step = strtol(scolon + 1, NULL, 10); if (step <= 0 || step > 100000000) return -10;
    *out_dir = dir; *out_step = (int)step; return 0;
}

/* 发送 HTTP 响应 */
static void http_send(int fd, const char *status, const char *ctype, const char *body) {
    char header[512]; int blen = body ? (int)strlen(body) : 0;
    int hlen = snprintf(header, sizeof(header),
                        "HTTP/1.1 %s\r\n"
                        "Content-Type: %s; charset=utf-8\r\n"
                        "Access-Control-Allow-Origin: *\r\n"
                        "Content-Length: %d\r\n"
                        "Connection: close\r\n\r\n",
                        status ? status : "200 OK",
                        ctype ? ctype : "text/plain",
                        blen);
    send(fd, header, hlen, 0); if (blen > 0) send(fd, body, blen, 0);
}

/* 内置最简 UI 页面，用于快速交互测试 */
static const char *UI_HTML =
"<!DOCTYPE html><html lang=\"zh\"><head><meta charset=\"utf-8\"><title>电机控制</title>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<style>body{font-family:sans-serif;margin:20px}label{display:inline-block;width:80px}button{margin:4px}input{margin:4px}</style>"
"</head><body>"
"<h3>三从站 TCP/HTTP 电机控制</h3>"
"<div><button id=\"btnConn\">连接</button><button id=\"btnDisc\">断开</button><span id=\"stat\">未连接</span></div>"
"<div><label>方向</label><select id=\"dir\"><option value=\"forward\">正转</option><option value=\"reverse\">反转</option></select></div>"
"<div><label>步长</label><input id=\"step\" type=\"number\" min=\"1\" value=\"5000\"></div>"
"<div><button id=\"btnExec\">执行</button><button id=\"btnStop\">停止</button><button id=\"btnDiag\">刷新诊断</button></div>"
"<div><pre id=\"out\"></pre></div>"
"<script>let connected=false;const out=document.getElementById('out');function log(t){out.textContent=t}\n"
"document.getElementById('btnConn').onclick=()=>{connected=true;document.getElementById('stat').textContent='已连接'};"
"document.getElementById('btnDisc').onclick=()=>{connected=false;document.getElementById('stat').textContent='未连接'};"
"document.getElementById('btnExec').onclick=async()=>{if(!connected){log('未连接');return;}\nconst dir=document.getElementById('dir').value;const step=parseInt(document.getElementById('step').value||'0');\ntry{const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({direction:dir,step:step})});const j=await r.text();log(j);}catch(e){log('错误:'+e)}};\n"
"document.getElementById('btnStop').onclick=async()=>{if(!connected){log('未连接');return;}\ntry{const r=await fetch('/stop',{method:'POST'});const t=await r.text();log(t);}catch(e){log('错误:'+e)}};\n"
"document.getElementById('btnDiag').onclick=async()=>{try{const r=await fetch('/diag');const t=await r.text();log(t);}catch(e){log('错误:'+e)}};\n"
"</script></body></html>";

/* 简易路由：GET /, /status, /diag；POST /control, /stop */
static void handle_http(int cfd, const char *req, int req_len) {
    if (!req || req_len <= 0) { http_send(cfd, "400 Bad Request", "text/plain", "bad request"); return; }
    if (strncmp(req, "GET ", 4) == 0) {
        const char *path = req + 4; const char *sp = strchr(path, ' '); if (!sp) { http_send(cfd, "400 Bad Request", "text/plain", "bad request"); return; }
        size_t plen = (size_t)(sp - path);
        if (plen == 1 && path[0] == '/') {  /* 首页返回内置 UI */
            http_send(cfd, "200 OK", "text/html", UI_HTML);
            return;
        }
        if (strncmp(path, "/status", plen) == 0) {
            char buf[256]; pthread_mutex_lock(&cmd_mutex); int dir = g_cmd.dir; int step = g_cmd.step; bool run = g_cmd.run; pthread_mutex_unlock(&cmd_mutex);
            snprintf(buf, sizeof(buf), "{\"run\":%s,\"dir\":%d,\"step\":%d,\"pos0\":%d,\"pos1\":%d,\"pos2\":%d}",
                     run?"true":"false", dir, step,
                     g_last_actual_pos[0], g_last_actual_pos[1], g_last_actual_pos[2]);
            http_send(cfd, "200 OK", "application/json", buf); return;
        }
        if (strncmp(path, "/diag", plen) == 0) {
            char buf[1024];
            uint16_t sw0 = EC_READ_U16(domain1_pd + input[0].statusword);
            uint16_t sw1 = EC_READ_U16(domain1_pd + input[1].statusword);
            uint16_t sw2 = EC_READ_U16(domain1_pd + input[2].statusword);
            int8_t md0 = EC_READ_S8(domain1_pd + input[0].workModeIn);
            int8_t md1 = EC_READ_S8(domain1_pd + input[1].workModeIn);
            int8_t md2 = EC_READ_S8(domain1_pd + input[2].workModeIn);
            int32_t fe0 = EC_READ_S32(domain1_pd + input[0].followingError);
            int32_t fe1 = EC_READ_S32(domain1_pd + input[1].followingError);
            int32_t fe2 = EC_READ_S32(domain1_pd + input[2].followingError);
            uint16_t ec0 = EC_READ_U16(domain1_pd + input[0].errorCode);
            uint16_t ec1 = EC_READ_U16(domain1_pd + input[1].errorCode);
            uint16_t ec2 = EC_READ_U16(domain1_pd + input[2].errorCode);
            uint16_t sec0 = EC_READ_U16(domain1_pd + input[0].servoErrorCode);
            uint16_t sec1 = EC_READ_U16(domain1_pd + input[1].servoErrorCode);
            uint16_t sec2 = EC_READ_U16(domain1_pd + input[2].servoErrorCode);
            uint32_t di0 = EC_READ_U32(domain1_pd + input[0].digitalInputs);
            uint32_t di1 = EC_READ_U32(domain1_pd + input[1].digitalInputs);
            uint32_t di2 = EC_READ_U32(domain1_pd + input[2].digitalInputs);
            uint16_t tpst0 = EC_READ_U16(domain1_pd + input[0].touchProbeStatus);
            uint16_t tpst1 = EC_READ_U16(domain1_pd + input[1].touchProbeStatus);
            uint16_t tpst2 = EC_READ_U16(domain1_pd + input[2].touchProbeStatus);
            int32_t tpp0 = EC_READ_S32(domain1_pd + input[0].touchProbePos);
            int32_t tpp1 = EC_READ_S32(domain1_pd + input[1].touchProbePos);
            int32_t tpp2 = EC_READ_S32(domain1_pd + input[2].touchProbePos);
            int32_t tgt0 = EC_READ_S32(domain1_pd + output[0].targetPosition);
            int32_t tgt1 = EC_READ_S32(domain1_pd + output[1].targetPosition);
            int32_t tgt2 = EC_READ_S32(domain1_pd + output[2].targetPosition);
            int32_t act0 = EC_READ_S32(domain1_pd + input[0].actualPosition);
            int32_t act1 = EC_READ_S32(domain1_pd + input[1].actualPosition);
            int32_t act2 = EC_READ_S32(domain1_pd + input[2].actualPosition);
            pthread_mutex_lock(&cmd_mutex);
            bool run = g_cmd.run; int dir = g_cmd.dir; int step = g_cmd.step;
            pthread_mutex_unlock(&cmd_mutex);
            int ack0 = (sw0 & 0x1000) ? 1 : 0;  /* bit12 Ack */
            int ack1 = (sw1 & 0x1000) ? 1 : 0;
            int ack2 = (sw2 & 0x1000) ? 1 : 0;
            int trg0 = (sw0 & 0x0400) ? 1 : 0;  /* bit10 Target reached */
            int trg1 = (sw1 & 0x0400) ? 1 : 0;
            int trg2 = (sw2 & 0x0400) ? 1 : 0;
            snprintf(buf, sizeof(buf),
                     "{\"status\":[%u,%u,%u],\"mode\":[%d,%d,%d],\"followingErr\":[%d,%d,%d],\"err\":[%u,%u,%u],\"servoErr\":[%u,%u,%u],\"din\":[%u,%u,%u],\"tpst\":[%u,%u,%u],\"tpp\":[%d,%d,%d],\"tgt\":[%d,%d,%d],\"act\":[%d,%d,%d],\"ack12\":[%d,%d,%d],\"targetReached10\":[%d,%d,%d],\"cmd\":{\"run\":%s,\"dir\":%d,\"step\":%d}}",
                     sw0, sw1, sw2, md0, md1, md2, fe0, fe1, fe2, ec0, ec1, ec2, sec0, sec1, sec2, di0, di1, di2, tpst0, tpst1, tpst2, tpp0, tpp1, tpp2, tgt0, tgt1, tgt2, act0, act1, act2, ack0, ack1, ack2, trg0, trg1, trg2, run?"true":"false", dir, step);
            http_send(cfd, "200 OK", "application/json", buf); return;
        }
        http_send(cfd, "404 Not Found", "text/plain", "not found"); return;
    }
    if (strncmp(req, "POST ", 5) == 0) {
        const char *path = req + 5; const char *sp = strchr(path, ' '); if (!sp) { http_send(cfd, "400 Bad Request", "text/plain", "bad request"); return; }
        size_t plen = (size_t)(sp - path); const char *hdr_end = strstr(req, "\r\n\r\n"); const char *body = hdr_end ? (hdr_end + 4) : NULL;
        if (strncmp(path, "/control", plen) == 0) { int dir = 0, step = 0; int rc = parse_control_json(body, &dir, &step); if (rc == 0) { set_motor_cmd(true, dir, step); http_send(cfd, "200 OK", "application/json", "{\"ok\":true}"); } else { http_send(cfd, "400 Bad Request", "application/json", "{\"ok\":false,\"error\":\"invalid json\"}"); } return; }
        if (strncmp(path, "/stop", plen) == 0) { set_motor_cmd(false, 0, 0); http_send(cfd, "200 OK", "application/json", "{\"ok\":true}"); return; }
        if (strncmp(path, "/shutdown", plen) == 0) { g_stop = 1; http_send(cfd, "200 OK", "application/json", "{\"ok\":true}"); return; }
        http_send(cfd, "404 Not Found", "text/plain", "not found"); return;
    }
    http_send(cfd, "405 Method Not Allowed", "text/plain", "method not allowed");
}

/* 简易 HTTP 服务器主循环 */
static void *http_server_thread(void *arg) {
    int port = (int)(intptr_t)arg; int sfd = socket(AF_INET, SOCK_STREAM, 0); if (sfd < 0) { perror("socket"); return NULL; }
    int opt = 1; setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); struct sockaddr_in addr = {0}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_ANY); addr.sin_port = htons(port);
    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); close(sfd); return NULL; }
    if (listen(sfd, 8) < 0) { perror("listen"); close(sfd); return NULL; }
    printf("HTTP server listening on port %d\n", port);
    for (;;) {
        struct sockaddr_in cli; socklen_t cl = sizeof(cli); int cfd = accept(sfd, (struct sockaddr *)&cli, &cl); if (cfd < 0) { if (errno == EINTR) continue; perror("accept"); break; }
        char buf[4096]; int n = recv(cfd, buf, sizeof(buf)-1, 0); if (n > 0) { buf[n] = '\0'; handle_http(cfd, buf, n); }
        close(cfd);
    }
    close(sfd); return NULL;
}

/* 打印域工作计数变化与工作状态 */
static void check_domain1_state(void) {
    ec_domain_state_t ds; ecrt_domain_state(domain1, &ds); if (ds.working_counter != domain1_state.working_counter) printf("Domain1: WC %u.\n", ds.working_counter); if (ds.wc_state != domain1_state.wc_state) printf("Domain1: State %u.\n", ds.wc_state); domain1_state = ds;
}

/* 打印主站状态变化 */
static void check_master_state(void) {
    ec_master_state_t ms; ecrt_master_state(master, &ms); if (ms.slaves_responding != master_state.slaves_responding) printf("%u slave(s).\n", ms.slaves_responding); if (ms.al_states != master_state.al_states) printf("AL states: 0x%02X.\n", ms.al_states); if (ms.link_up != master_state.link_up) printf("Link is %s.\n", ms.link_up ? "up" : "down"); master_state = ms;
}

/* 打印各从站在线、操作态变化 */
static void check_slave_config_states(void) {
    ec_slave_config_state_t s; ecrt_slave_config_state(sc0, &s); if (s.al_state != sc0_state.al_state) printf("slave0: State 0x%02X.\n", s.al_state); if (s.online != sc0_state.online) printf("slave0: %s.\n", s.online ? "online" : "offline"); if (s.operational != sc0_state.operational) printf("slave0: %soperational.\n", s.operational ? "" : "Not "); sc0_state = s;
    ecrt_slave_config_state(sc1, &s); if (s.al_state != sc1_state.al_state) printf("slave1: State 0x%02X.\n", s.al_state); if (s.online != sc1_state.online) printf("slave1: %s.\n", s.online ? "online" : "offline"); if (s.operational != sc1_state.operational) printf("slave1: %soperational.\n", s.operational ? "" : "Not "); sc1_state = s;
    ecrt_slave_config_state(sc2, &s); if (s.al_state != sc2_state.al_state) printf("slave2: State 0x%02X.\n", s.al_state); if (s.online != sc2_state.online) printf("slave2: %s.\n", s.online ? "online" : "offline"); if (s.operational != sc2_state.operational) printf("slave2: %soperational.\n", s.operational ? "" : "Not "); sc2_state = s;
}

/*
 * 实时循环：
 * 1) 接收/处理过程数据
 * 2) 推进 CiA-402 状态机，进入 Operation enabled
 * 3) 在 CSP 下更新目标位置并写控制字
 * 4) 周期诊断输出
 */
void cyclic_task() {
    static uint32_t time_cnt[3] = {0, 0, 0};
    static bool servo_enabled[3] = {false, false, false};
    static int32_t start_pos[3] = {0, 0, 0};
    static int debug_counter = 0;
    static int csp_warmup[3] = {0, 0, 0};
    static int32_t csp_target[3] = {0, 0, 0};
    ecrt_master_application_time(master, monotonic_ns());
    ecrt_master_receive(master); ecrt_domain_process(domain1);
    ecrt_master_sync_slave_clocks(master);
    check_domain1_state(); check_master_state(); check_slave_config_states();
    debug_counter++; if (debug_counter % 1000 == 0) printf("Debug en:%d,%d,%d\n", servo_enabled[0], servo_enabled[1], servo_enabled[2]);
    for (int i = 0; i < 3; ++i) {
        uint16_t status_i = EC_READ_U16(domain1_pd + input[i].statusword);
        uint16_t control_i = 0;
        if (!servo_enabled[i]) {
            uint8_t current_mode = EC_READ_S8(domain1_pd + input[i].workModeIn);
            /* 低 7 位状态机分支 */
            switch (status_i & 0x6F) {
                case 0x00: control_i = 0x06; break;
                case 0x40: control_i = 0x06; break;
                case 0x21:
                    control_i = 0x07;  /* Switch on */
                    start_pos[i] = EC_READ_S32(domain1_pd + input[i].actualPosition);
                    EC_WRITE_S32(domain1_pd + output[i].targetPosition, start_pos[i]);
                    break;
                case 0x23: control_i = 0x0F; break;
                case 0x27: control_i = 0x0F; servo_enabled[i] = true; csp_warmup[i] = 10; csp_target[i] = EC_READ_S32(domain1_pd + input[i].actualPosition); break;
                default: control_i = 0x06; break;
            }
            /* Fault 状态下清错 */
            if ((status_i & 0x0040) && !(status_i & 0x0001)) {
                EC_WRITE_U16(domain1_pd + output[i].controlWord, 0x0000);
                EC_WRITE_U16(domain1_pd + output[i].controlWord, 0x0080);
            }
            if (debug_counter % 500 == 0) {
                int ack = (status_i & 0x1000) ? 1 : 0;
                int trg = (status_i & 0x0400) ? 1 : 0;
                int32_t ap = EC_READ_S32(domain1_pd + input[i].actualPosition);
                printf("[EN%d] status:0x%04X mode:%d ctrl:0x%04X ack12:%d trg10:%d act:%d\n", i, status_i, current_mode, control_i, ack, trg, ap);
                g_last_actual_pos[i] = ap;
            }
            EC_WRITE_U16(domain1_pd + output[i].controlWord, control_i);
            EC_WRITE_S8(domain1_pd + output[i].workModeOut, WORK_MODE);
        } else {
            time_cnt[i]++;
            if (time_cnt[i] == 1) {
                /* 初次进入运行，目标对齐到实际位置，并置“新设定值”位 */
                start_pos[i] = EC_READ_S32(domain1_pd + input[i].actualPosition);
                csp_target[i] = start_pos[i];
                EC_WRITE_S32(domain1_pd + output[i].targetPosition, csp_target[i]);
                EC_WRITE_U16(domain1_pd + output[i].controlWord, 0x0F);
            }
            else {
                pthread_mutex_lock(&cmd_mutex); int dir = g_cmd.dir; int step = g_cmd.step; bool run = g_cmd.run; pthread_mutex_unlock(&cmd_mutex);
                int delta = (run ? (dir * step) : 0);
                if (delta > MAX_DELTA_PER_CYCLE) delta = MAX_DELTA_PER_CYCLE;
                if (delta < -MAX_DELTA_PER_CYCLE) delta = -MAX_DELTA_PER_CYCLE;
                if (csp_warmup[i] > 0) {
                    csp_target[i] = EC_READ_S32(domain1_pd + input[i].actualPosition);
                    csp_warmup[i]--;
                } else {
                    csp_target[i] += delta;
                }
                EC_WRITE_S32(domain1_pd + output[i].targetPosition, csp_target[i]);
                EC_WRITE_U16(domain1_pd + output[i].controlWord, 0x0F);
            }
            EC_WRITE_S8(domain1_pd + output[i].workModeOut, WORK_MODE);
            if (time_cnt[i] % 500 == 0) {
                int32_t actual_pos_i = EC_READ_S32(domain1_pd + input[i].actualPosition);
                uint16_t sw_i = EC_READ_U16(domain1_pd + input[i].statusword);
                int8_t md_i = EC_READ_S8(domain1_pd + input[i].workModeIn);
                int32_t fe_i = EC_READ_S32(domain1_pd + input[i].followingError);
                uint16_t ec_i = EC_READ_U16(domain1_pd + input[i].errorCode);
                uint16_t sec_i = EC_READ_U16(domain1_pd + input[i].servoErrorCode);
                uint32_t di_i = EC_READ_U32(domain1_pd + input[i].digitalInputs);
                uint16_t tpst_i = EC_READ_U16(domain1_pd + input[i].touchProbeStatus);
                int32_t tpp_i = EC_READ_S32(domain1_pd + input[i].touchProbePos);
                printf("[M%d] tgt:%d act:%d sw:0x%04X mode:%d fe:%d err:0x%04X servoErr:0x%04X din:0x%08X tpst:0x%04X tpp:%d\n",
                       i, EC_READ_S32(domain1_pd + output[i].targetPosition), actual_pos_i, sw_i, md_i, fe_i, ec_i, sec_i, di_i, tpst_i, tpp_i);
                g_last_actual_pos[i] = actual_pos_i;
            }
        }
    }
    /* 排队并发送过程数据 */
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/*
 * 程序入口：
 * - 初始化主站与域
 * - 配置从站 PDO/同步管理器
 * - 设置基本 SDO 参数（插值周期、速度轮廓等）
 * - 启动 HTTP 服务并进入 1ms/4ms 控制循环
 */
int main(int argc, char **argv) {
    printf("Starting three-slave CSP motor controller\n");
    master = ecrt_request_master(0);
    if (!master) exit(EXIT_FAILURE);
    domain1 = ecrt_master_create_domain(master);
    if (!domain1) exit(EXIT_FAILURE);
    if (!(sc0 = ecrt_master_slave_config(master, SLV0, VENDOR_PRODUCT))) { fprintf(stderr, "Failed to get slave0 config\n"); exit(EXIT_FAILURE); }
    if (!(sc1 = ecrt_master_slave_config(master, SLV1, VENDOR_PRODUCT))) { fprintf(stderr, "Failed to get slave1 config\n"); exit(EXIT_FAILURE); }
    if (!(sc2 = ecrt_master_slave_config(master, SLV2, VENDOR_PRODUCT))) { fprintf(stderr, "Failed to get slave2 config\n"); exit(EXIT_FAILURE); }
    (void)ecrt_slave_config_sdo8(sc0, 0x60C2, 2, (uint8_t)-3);
    (void)ecrt_slave_config_sdo8(sc0, 0x60C2, 1, 4);
    (void)ecrt_slave_config_sdo8(sc1, 0x60C2, 2, (uint8_t)-3);
    (void)ecrt_slave_config_sdo8(sc1, 0x60C2, 1, 4);
    (void)ecrt_slave_config_sdo8(sc2, 0x60C2, 2, (uint8_t)-3);
    (void)ecrt_slave_config_sdo8(sc2, 0x60C2, 1, 4);
    (void)ecrt_slave_config_sdo32(sc0, 0x6081, 0, 100000); // Profile velocity limit
    (void)ecrt_slave_config_sdo32(sc1, 0x6081, 0, 100000);
    (void)ecrt_slave_config_sdo32(sc2, 0x6081, 0, 100000);
    (void)ecrt_slave_config_sdo32(sc0, 0x6083, 0, 50000);  // Acceleration
    (void)ecrt_slave_config_sdo32(sc1, 0x6083, 0, 50000);
    (void)ecrt_slave_config_sdo32(sc2, 0x6083, 0, 50000);
    (void)ecrt_slave_config_sdo32(sc2, 0x6084, 0, 50000);  // Deceleration

    ecrt_master_select_reference_clock(master, sc0);
    (void)ecrt_slave_config_dc(sc0, 0x0300, DC_SYNC0_PERIOD_NS, 0, 0, 0);
    (void)ecrt_slave_config_dc(sc1, 0x0300, DC_SYNC0_PERIOD_NS, 0, 0, 0);
    (void)ecrt_slave_config_dc(sc2, 0x0300, DC_SYNC0_PERIOD_NS, 0, 0, 0);
    (void)ecrt_slave_config_sdo32(sc1, 0x6084, 0, 50000);
    (void)ecrt_slave_config_sdo32(sc2, 0x6084, 0, 50000);
    
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc0, EC_END, device_syncs)) { fprintf(stderr, "Failed to configure slave0 PDOs\n"); exit(EXIT_FAILURE);} 
    if (ecrt_slave_config_pdos(sc1, EC_END, device_syncs)) { fprintf(stderr, "Failed to configure slave1 PDOs\n"); exit(EXIT_FAILURE);} 
    if (ecrt_slave_config_pdos(sc2, EC_END, device_syncs)) { fprintf(stderr, "Failed to configure slave2 PDOs\n"); exit(EXIT_FAILURE);} 
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) { fprintf(stderr, "PDO entry registration failed\n"); exit(EXIT_FAILURE); }
    printf("Activating master...\n");
    if (ecrt_master_activate(master)) exit(EXIT_FAILURE);
    if (!(domain1_pd = ecrt_domain_data(domain1))) exit(EXIT_FAILURE);
    printf("HTTP server starting...\n");
    if (pthread_create(&http_thread, NULL, http_server_thread, (void *)(intptr_t)http_port) != 0) { perror("pthread_create"); }
    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler); signal(SIGHUP, sig_handler);
    while (!g_stop) {
        usleep(TASK_FREQUENCY);
        cyclic_task();
    }
    pthread_cancel(http_thread);
    pthread_join(http_thread, NULL);
    ecrt_release_master(master);
    return EXIT_SUCCESS;
}

/*{"status":[567,567,567],"mode":[8,8,8],"followingErr":[-3,0,0],"err":[0,0,0],"servoErr":[0,0,0],"din":[6291459,6291459,6291459],"tpst":[0,0,0],"tpp":[0,0,0],"tgt":[5882006,5881998,5882000],"act":[12,-2,1],"ack12":[0,0,0],"targetReached10":[0,0,0],"cmd":{"run":true,"dir":1,"step":1000}}*/
