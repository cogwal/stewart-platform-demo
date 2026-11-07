/* Fake bus and fake TMCM-1270 drives for unit-testing the master core without CAN.
 * Deliberately small: the faithful emulator is the Python one (sim/drive), used in the e2e tests.
 * This one covers the CiA 402 state machine, pv/hm/pp/csp, stall at the end stop, NMT and EMCY. */
#ifndef FAKE_BUS_H
#define FAKE_BUS_H

#include <stdbool.h>
#include <stdint.h>

#include "sp_app.h"

typedef enum { FD_SOD, FD_RTSO, FD_SO, FD_OE, FD_QSA, FD_FAULT } fd_state_t;

typedef struct {
    uint8_t node;
    bool powered;
    bool hb_alive;         /* false: heartbeat frames lost (cable) */
    int nmt;               /* SP_NMTS_* */
    fd_state_t state;
    int8_t mode;
    uint16_t cw;
    double counts;         /* open-loop step counter (6064h) */
    double truth_mm;       /* where the rod really is */
    double vel_pps;
    int32_t pv_target;
    int32_t pp_target;
    bool pp_moving, pp_ack, homing_attained;
    uint8_t current;
    bool fault_on_stall;
    bool no_stall_detect;  /* stallGuard never triggers: homing must end by timeout or load */
    bool stalled;
    uint32_t sdo_writes;
    bool bootup_pending;
    int emcy_pending;      /* code, 0 = none */
} fdrive_t;

typedef struct {
    sp_app_t app;
    fdrive_t d[SP_MAX_LEGS];
    uint8_t n;
    uint32_t t_ms;
    /* outstanding SDO, completed on the next tick */
    bool sdo_active;
    uint8_t sdo_node;
    bool sdo_write;
    uint16_t sdo_index;
    uint8_t sdo_sub;
    uint32_t sdo_value;
    bool sdo_hang;         /* adapter never answers (test the core's guard) */
    int log_level;         /* print log lines up to this level, -1 none */
    uint32_t frames;
} fake_t;

void fake_init(fake_t* f, uint8_t n_legs, double rod_mm, bool auto_home);
void fake_run(fake_t* f, uint32_t ms);
bool fake_run_until(fake_t* f, uint32_t max_ms, bool (*cond)(fake_t*));
void fake_power_cycle(fake_t* f, uint8_t leg);
fdrive_t* fake_drive_by_node(fake_t* f, uint8_t node);

#endif
