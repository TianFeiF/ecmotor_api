#include <ecrt.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>

// EtherCAT 从站信息
#define VENDOR_ID       0x00001097 // 示例Vendor ID（请替换为你的设备实际ID）
#define PRODUCT_CODE    0x00002406 // 示例Product Code（请替换为你的设备实际ID）
#define SLAVE_POS       0

// 全局变量
static ec_master_t *master = NULL;
static ec_domain_t *domain = NULL;
static ec_slave_config_t *sc = NULL;
static uint8_t *domain_pd = NULL;

uint control_word;
uint target_pos;
uint target_velocity;
uint target_tor;
uint op_mode;
uint resv1;

uint status_word;
uint actual_pos;
uint actual_velocity;
uint actual_tor;
uint op_mode_disp;
uint error_code;
uint resv2;

static int run = 1;

void signal_handler(int sig) {
    run = 0;
}

// PDO entry 索引
enum {
    CONTROL_WORD = 0,
    TARGET_POS,
    TARGET_VELOCITY,
    TARGET_TOR,
    OP_MODE,
    RESV1,
    STATUS_WORD,
    ACTUAL_POS,
    ACTUAL_VELOCITY,
    ACTUAN_TOR,
    OP_MODE_DIS,
    RESV2,
    NUM_ENTRIES
};

// static ec_pdo_entry_reg_t domain_regs[] = {
//     {SLAVE_POS, VENDOR_ID, PRODUCT_CODE, 0x6040, 0x00, &control_word, NULL},
//     {SLAVE_POS, VENDOR_ID, PRODUCT_CODE, 0x607A, 0x00, &target_pos, NULL},
//     {SLAVE_POS, VENDOR_ID, PRODUCT_CODE, 0x60FF, 0x00, &target_velocity, NULL},
//     {SLAVE_POS, VENDOR_ID, PRODUCT_CODE, 0x6041, 0x00, &status_word, NULL},
//     {SLAVE_POS, VENDOR_ID, PRODUCT_CODE, 0x606C, 0x00, &actual_velocity, NULL},
//     {}
// };

// PDO entries
static ec_pdo_entry_info_t slave_pdo_entries[] = {
    {0x6040, 0x00, 16}, // Control word
    {0x607A, 0x00, 32}, // Target pos
    {0x60FF, 0x00, 32}, // Target velocity
    {0x6071, 0x00, 16}, // Control word
    {0x6060, 0x00, 8}, // Control word
    {0x60C2, 0x00, 8}, // resv

    {0x6041, 0x00, 16}, // Status word
    {0x6064, 0x00, 32}, // Actual pos
    {0x606C, 0x00, 32}, // Actual velocity
    {0x6077, 0x00, 16}, // Actual tor
    {0x6061, 0x00, 8}, // op mode disp
    {0x603F, 0x00, 16}, // error code
    {0x2026, 0x00, 8}, // resv2
};

// RxPDO
static ec_pdo_info_t slave_rx_pdo[] = {
    {0x1600, 6, slave_pdo_entries},
};

// TxPDO
static ec_pdo_info_t slave_tx_pdo[] = {
    {0x1A00, 7, &slave_pdo_entries[6]},
};

// Sync managers
const static ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_rx_pdo, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_tx_pdo, EC_WD_DISABLE},
    {0xff}
};

ec_pdo_entry_reg_t domain_regs[] = {
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x6040, 0x00, &control_word, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x607A, 0x00, &target_pos, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x60FF, 0x00, &target_velocity, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x6071, 0x00, &target_tor, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x6060, 0x00, &op_mode, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x60C2, 0x00, &resv1, NULL},

    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x6041, 0x00, &status_word, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x6064, 0x00, &actual_pos, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x606C, 0x00, &actual_velocity, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x6077, 0x00, &actual_tor, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x6061, 0x00, &op_mode_disp, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x603F, 0x00, &error_code, NULL},
    {0, 0,VENDOR_ID, PRODUCT_CODE, 0x2026, 0x00, &resv2, NULL},
    {} // 必须以空项结尾
};

// 获取 entry 对应数据指针
uint16_t *get_control_word() {
    return (uint16_t *)(domain_pd + control_word);
}
int32_t *get_target_pos() {
    return (int32_t *)(domain_pd + target_pos);
}
int32_t *get_target_velocity() {
    return (int32_t *)(domain_pd + target_velocity);
}

uint16_t *get_status_word() {
    return (uint16_t *)(domain_pd + status_word);
}
int32_t *get_actual_velocity() {
    return (int32_t *)(domain_pd + actual_velocity);
}
int32_t *get_actual_pos() {
    return (int32_t *)(domain_pd + actual_pos);
}
int8_t *get_op_mode() {
    return (int8_t *)(domain_pd + op_mode);
}
int8_t *get_resv1() {
    return (int8_t *)(domain_pd + resv1);
}
// 小端转换（如果确认从站数据是小端）
int32_t read_le_int32(uint8_t *data)
{
  return static_cast<int32_t>(
    (data[0]) |
    (data[1] << 8) |
    (data[2] << 16) |
    (data[3] << 24));
}

int16_t read_le_int16(const uint8_t *data) {
  return static_cast<int16_t>(
    (data[0]) |
    (data[1] << 8));
}
// 主函数
int main() {
    signal(SIGINT, signal_handler);

    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "Failed to get master.\n");
        return -1;
    }

    domain = ecrt_master_create_domain(master);
    if (!domain) {
        fprintf(stderr, "Failed to create domain.\n");
        return -1;
    }

    sc = ecrt_master_slave_config(master, 0, SLAVE_POS, VENDOR_ID, PRODUCT_CODE);
    if (!sc) {
        fprintf(stderr, "Failed to get slave config.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, slave_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
        fprintf(stderr, "Failed to register PDO entries.\n");
        return -1;
    }

    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master.\n");
        return -1;
    }

    domain_pd = ecrt_domain_data(domain);

    printf("Started main loop.\n");
    int step = 50;
    //printf("start pos%d.\n",startPos);
    bool brun_enable = false;

	int32_t startPos = 0;

    uint8_t op = 8;
    uint8_t tmp = 1;
    
    while (run) {
        memcpy(domain_pd+op_mode, &op, sizeof(uint8_t));
        memcpy(domain_pd+resv1, &tmp, sizeof(uint8_t));
        ecrt_master_receive(master);
        ecrt_domain_process(domain);

       //*get_op_mode() = 8; // Velocity mode
        //*get_target_velocity() = 400000; // 设置目标速度
        
        // 获取状态
        uint16_t status = *get_status_word();
        // 状态机转换
        uint16_t control = 0;
        switch (status & 0x6F) {
                   
            case 0x00: // Not ready to switch on
                control = 0x06; // Shutdown
                break;
            case 0x40: // Switch on disabled
                control = 0x06; // Shutdown
                break;
            case 0x21: // Ready to switch on
                control = 0x07; // Switch on
                startPos = read_le_int32(domain_pd+actual_pos);
                printf("startPos:%d.\n",startPos);
                memcpy(domain_pd+target_pos, &startPos, sizeof(int32_t));
                break;
            case 0x23: // Switched on
                control = 0x0F; // Enable operation
                break;
            case 0x27: // Operation enabled
                control = 0x0F; // Maintain
                brun_enable = true;
                break;
            default:
                control = 0x06; // Fallback
                break;
        }
        *get_control_word() = control;
        
        if (brun_enable){
            startPos += step;
            *get_target_pos() = startPos;
            //printf("target pos%d.\n",startPos);
        }



        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        usleep(1000); // 1ms 循环
    }

    return 0;
}
