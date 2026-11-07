/* spmaster: the master core on CANopenLinux (CO_main_basic.c provides main() and the CANopenNode
 * loop, single-threaded; this file implements its application hooks). Operator interface over
 * UDP JSON, the same protocol as the Python test master (sim/master/testmaster.py).
 *
 *   build/master/spmaster -i 127 -s build/master-storage/ vcan0
 *
 * Environment:
 *   SP_LEGS=6               legs to use (first n of SP_NODES)
 *   SP_NODES=1,2,3,4,5,6    drive node ids (default: contract)
 *   SP_AUTO_HOME=1          boot and home at start
 *   SP_CMD_PORT=5005        UDP commands in (127.0.0.1)
 *   SP_STATUS_PORT=5006     UDP status out, every 100 ms
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "CO_application.h"
#include "sp_co_adapter.h"

static sp_app_t g_app;
static sp_co_t g_ad;
static int g_cmd_fd = -1, g_status_fd = -1;
static struct sockaddr_in g_status_addr;
static uint32_t g_status_us;

static int env_int(const char* name, int def) {
    const char* v = getenv(name);
    return v && *v ? atoi(v) : def;
}

static void log_cb(void* ctx, int level, const char* text) {
    static const char* const lv[] = {"ERROR", "WARN", "INFO"};
    struct timespec ts;
    (void)ctx;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    printf("spmaster %6ld.%03ld %-5s %s\n", (long)ts.tv_sec % 100000, ts.tv_nsec / 1000000, lv[level < 0 ? 0 : level > 2 ? 2 : level],
           text);
    fflush(stdout);
}

static int udp_socket(uint16_t bind_port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return -1;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    if (bind_port) {
        struct sockaddr_in a = {0};
        int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        a.sin_family = AF_INET;
        a.sin_port = htons(bind_port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(fd, (struct sockaddr*)&a, sizeof a) != 0) {
            fprintf(stderr, "spmaster: bind 127.0.0.1:%u: %s\n", bind_port, strerror(errno));
            close(fd);
            return -1;
        }
    }
    return fd;
}

CO_ReturnError_t app_programStart(uint16_t* bitRate, uint8_t* nodeId, uint32_t* errInfo) {
    static const uint8_t contract_nodes[SP_MAX_LEGS] = SP_DRIVE_NODE_IDS;
    uint8_t nodes[SP_MAX_LEGS];
    sp_port_t port;
    (void)bitRate;
    (void)errInfo;
    memcpy(nodes, contract_nodes, sizeof nodes);
    const char* list = getenv("SP_NODES");
    if (list && *list) {
        memset(nodes, 0, sizeof nodes);
        char buf[64];
        snprintf(buf, sizeof buf, "%s", list);
        int k = 0;
        for (char* tok = strtok(buf, ","); tok && k < SP_MAX_LEGS; tok = strtok(NULL, ",")) {
            nodes[k++] = (uint8_t)atoi(tok);
        }
    }
    int legs = env_int("SP_LEGS", SP_CONTRACT_LEGS);
    if (legs < 1 || legs > SP_MAX_LEGS) {
        legs = SP_CONTRACT_LEGS;
    }
    if (*nodeId == 0 || *nodeId > 127) {
        *nodeId = SP_MASTER_NODE_ID;
    }
    sp_co_configure_od(nodes, (uint8_t)legs, *nodeId);
    sp_co_port(&g_ad, &port, log_cb);
    sp_app_init(&g_app, &port, nodes, (uint8_t)legs, *nodeId, env_int("SP_AUTO_HOME", 1) != 0);
    sp_co_init(&g_ad, &g_app, nodes, (uint8_t)legs);

    g_cmd_fd = udp_socket((uint16_t)env_int("SP_CMD_PORT", 5005));
    g_status_fd = udp_socket(0);
    memset(&g_status_addr, 0, sizeof g_status_addr);
    g_status_addr.sin_family = AF_INET;
    g_status_addr.sin_port = htons((uint16_t)env_int("SP_STATUS_PORT", 5006));
    g_status_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    char line[96];
    snprintf(line, sizeof line, "master node %u, %d legs, contract %s, geometry %s", *nodeId, legs,
             SP_CONTRACT_VERSION, SP_GEOMETRY_VERSION);
    log_cb(NULL, 2, line);
    return CO_ERROR_NO;
}

void app_communicationReset(CO_t* co) { sp_co_on_comm_reset(&g_ad, co); }

void app_programEnd(void) {
    if (g_cmd_fd >= 0) close(g_cmd_fd);
    if (g_status_fd >= 0) close(g_status_fd);
    log_cb(NULL, 2, "exit");
}

void app_programAsync(CO_t* co, uint32_t timer1usDiff) {
    char buf[1024];
    (void)co;
    if (g_cmd_fd >= 0) {
        ssize_t n;
        while ((n = recv(g_cmd_fd, buf, sizeof buf - 1, 0)) > 0) {
            buf[n] = '\0';
            if (sp_app_command(&g_app, buf) != 0) {
                log_cb(NULL, 1, "bad command");
            }
        }
    }
    sp_co_process(&g_ad, timer1usDiff);
    g_status_us += timer1usDiff;
    if (g_status_us >= 100000u && g_status_fd >= 0) {
        static char status[4096];
        g_status_us = 0;
        int len = sp_app_status_json(&g_app, status, sizeof status);
        sendto(g_status_fd, status, (size_t)len, 0, (struct sockaddr*)&g_status_addr, sizeof g_status_addr);
    }
}

void app_programRt(CO_t* co, uint32_t timer1usDiff) {
    (void)co;
    (void)timer1usDiff;
}
