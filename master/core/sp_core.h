/* Stewart platform master core: drive bring-up, homing, fault handling and csp streaming for up to
 * six TMCM-1270 legs, following contract/drive-contract.yaml (via the generated sp_config_gen.h).
 *
 * Portable C99, no dynamic memory, no OS calls, no CANopen stack calls: everything goes through
 * sp_port_t, and time is passed in. The same file runs on the PC (CANopenLinux) and the TTC2000 ECU.
 *
 * Call pattern (one thread / task):
 *   each loop:   fill sp_core_leg_in() for every leg from the bus, then sp_core_step(dt_us)
 *   each SYNC:   sp_core_set_leg_targets() (from sp_motion), then sp_core_on_sync()
 *   on events:   sp_core_on_bootup(), sp_core_on_emcy(), sp_core_sdo_done()
 *   operator:    sp_core_cmd_home(), sp_core_cmd_stop(), sp_core_cmd_resume()
 */
#ifndef SP_CORE_H
#define SP_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "sp_config_gen.h"
#include "sp_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* NMT states as reported in heartbeats. */
enum { SP_NMTS_UNKNOWN = -1, SP_NMTS_BOOTUP = 0, SP_NMTS_STOPPED = 4, SP_NMTS_OPERATIONAL = 5, SP_NMTS_PREOP = 127 };

typedef enum {
    SP_PH_BOOT,    /* bringing drives up (reset, identify, configure, NMT start) */
    SP_PH_IDLE,    /* drives configured, not homed: waiting for Home */
    SP_PH_HOMING,  /* all legs: retract to the end stop, set home, park */
    SP_PH_RUN,     /* homed, following leg targets in csp */
    SP_PH_STOPPED, /* quick-stopped (operator or stop_all EMCY); homing kept: Resume or Home */
    SP_PH_FAULT    /* homing lost or failed: Home needed */
} sp_phase_t;

typedef enum {
    SP_BOOT_OFF,       /* leg not used */
    SP_BOOT_RESET,     /* send NMT reset node */
    SP_BOOT_WAIT,      /* wait for boot-up */
    SP_BOOT_IDENTIFY,  /* read 1018h */
    SP_BOOT_PREPARE,   /* bring the drive to SWITCH ON DISABLED over SDO */
    SP_BOOT_CONFIGURE, /* startup SDO table */
    SP_BOOT_START,     /* NMT start, wait for OPERATIONAL */
    SP_BOOT_READY,
    SP_BOOT_LOST,      /* heartbeat lost while ready */
    SP_BOOT_ERROR      /* retried after a delay */
} sp_boot_t;

/* Bus observations for one leg, written by the adapter before each sp_core_step(). */
typedef struct {
    bool hb_active;      /* heartbeat consumer: heartbeats arriving */
    int16_t nmt;         /* last NMT state from the heartbeat, SP_NMTS_UNKNOWN if none */
    uint16_t statusword; /* 6041h from TPDO1/TPDO2 */
    int32_t position;    /* 6064h from TPDO1, counts */
    int8_t mode_display; /* 6061h from TPDO2 */
} sp_leg_in_t;

typedef struct {
    uint32_t t_ms;
    uint8_t node;
    uint16_t code;
    uint8_t byte1;
    uint8_t cls; /* sp_emcy_class_t */
} sp_emcy_log_t;

#define SP_EMCY_LOG_LEN 8
#define SP_REASON_LEN 64

typedef struct {
    sp_leg_in_t in;
    uint8_t node;
    bool bootup;        /* boot-up seen since last handled */

    sp_boot_t boot;
    uint8_t boot_sub;
    uint16_t boot_i;
    uint32_t boot_t0;
    uint32_t revision;

    uint8_t motion;     /* internal motion state */
    uint32_t motion_t0;
    uint32_t poll_t0;
    uint8_t en;         /* enable sub-state */
    uint32_t en_t0;
    int8_t en_mode;
    bool stall_emcy;
    uint8_t end_reason; /* homing: how the end stop was found */
    uint16_t load;      /* last 2101h value */

    bool sdo_pending;   /* this leg owns the outstanding SDO */
    bool sdo_done;
    uint32_t sdo_abort;
    uint32_t sdo_value;

    float target_mm;    /* requested (from sp_motion or the operator), clamped to soft limits */
    float x_mm;         /* rate-limited csp position, streamed to the drive */
    float v_mm_s;
} sp_leg_t;

typedef struct {
    sp_port_t port;
    uint8_t n_legs;
    uint8_t master_node;
    uint32_t now_ms;
    uint32_t us_acc;

    sp_phase_t phase;
    bool homed;
    bool home_requested;
    bool auto_home;
    bool sdo_busy;
    uint32_t sdo_t0;
    char reason[SP_REASON_LEN];

    sp_leg_t legs[SP_MAX_LEGS];
    sp_emcy_log_t emcy[SP_EMCY_LOG_LEN];
    uint8_t emcy_n;     /* total received, wraps */
    uint32_t syncs;
} sp_core_t;

/* node_ids: SP_MAX_LEGS entries (0 = unused); n_legs: how many of them to use. */
void sp_core_init(sp_core_t* core, const sp_port_t* port, const uint8_t* node_ids, uint8_t n_legs,
                  uint8_t master_node, bool auto_home);

sp_leg_in_t* sp_core_leg_in(sp_core_t* core, uint8_t leg);
void sp_core_step(sp_core_t* core, uint32_t dt_us);
void sp_core_on_sync(sp_core_t* core);

void sp_core_on_bootup(sp_core_t* core, uint8_t node);
void sp_core_on_emcy(sp_core_t* core, uint8_t node, uint16_t code, uint8_t reg, uint8_t byte1);
void sp_core_sdo_done(sp_core_t* core, uint32_t abort_code, uint32_t value);

/* Leg targets in mm from the retracted end stop. Clamped to the soft limits. */
void sp_core_set_leg_targets(sp_core_t* core, const float* mm, uint8_t count);

bool sp_core_cmd_home(sp_core_t* core);
void sp_core_cmd_stop(sp_core_t* core, const char* why);
bool sp_core_cmd_resume(sp_core_t* core);

/* Introspection (status display, tests). */
const char* sp_phase_name(sp_phase_t phase);
const char* sp_boot_name(sp_boot_t boot);
const char* sp_emcy_class_name(uint8_t cls);
const char* sp_leg_state_name(uint16_t statusword);
uint8_t sp_emcy_classify(uint16_t code, uint8_t byte1);
float sp_leg_actual_mm(const sp_core_t* core, uint8_t leg);
bool sp_core_all_legs_in_csp(const sp_core_t* core);

#ifdef __cplusplus
}
#endif

#endif /* SP_CORE_H */
