#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "motor_api.h"

static volatile sig_atomic_t stop = 0;
static void sig_handler(int s){ (void)s; stop = 1; }

int main(int argc, char **argv) {
    const char *eni = "motor_api/doc/HCFAX3E.xml";
    if (argc > 1) eni = argv[1];
    struct motor_api_handle *h = NULL; uint16_t slaves = 0;
    if (motor_api_create(eni, 4000, &slaves, &h) != MA_OK || !h) { fprintf(stderr, "motor_api_create failed\n"); return 1; }
    printf("motor_api created, slaves=%u\n", slaves);
    /* 固定500步长，正向运行 */
    motor_api_set_command(h, true, 1, 500);
    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler);
    while (!stop) { motor_api_run_once(h); usleep(4000); }
    /* 退出前停止并销毁 */
    motor_api_set_command(h, false, 0, 0);
    motor_api_destroy(h);
    return 0;
}
