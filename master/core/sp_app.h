/* Application glue shared by the PC and the ECU: core + motion, operator commands (JSON text, one
 * object per message) and a JSON status report. The transport is the platform's business: UDP on
 * the PC (master/linux), whatever the ECU offers later (debug CAN, UART, or local I/O directly via
 * the sp_motion_set_*() calls).
 *
 * Commands:
 *   {"type":"home"} {"type":"stop"} {"type":"resume"}
 *   {"type":"source","source":"pose|joystick|imu|demo|legs"}
 *   {"type":"pose","x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0}      mm / deg, missing keys = 0
 *   {"type":"joystick","x":0.2,"y":-0.5}                               -1..1
 *   {"type":"imu","roll":3.0,"pitch":-2.0,"yaw":0}                      deg
 *   {"type":"legs","mm":[50,50,50,50,50,50]}
 *   {"type":"setpoint","mm":42.0}            all legs to one position (single-leg web view)
 */
#ifndef SP_APP_H
#define SP_APP_H

#include <stddef.h>
#include <stdint.h>

#include "sp_core.h"
#include "sp_motion.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    sp_core_t core;
    sp_motion_t motion;
    sp_phase_t last_phase;
    uint32_t commands;
    uint32_t bad_commands;
} sp_app_t;

void sp_app_init(sp_app_t* app, const sp_port_t* port, const uint8_t* node_ids, uint8_t n_legs, uint8_t master_node,
                 bool auto_home);
void sp_app_step(sp_app_t* app, uint32_t dt_us);
void sp_app_on_sync(sp_app_t* app);
/* Returns 0 if the command was understood. */
int sp_app_command(sp_app_t* app, const char* json);
/* Returns the length written (truncated JSON if n is too small; 4096 is plenty for 6 legs). */
int sp_app_status_json(const sp_app_t* app, char* buf, size_t n);

/* Minimal JSON field readers (flat objects; used by sp_app_command, exposed for tests). */
int sp_json_string(const char* json, const char* key, char* out, size_t n);
int sp_json_number(const char* json, const char* key, float* out);
int sp_json_numbers(const char* json, const char* key, float* out, int max);

#ifdef __cplusplus
}
#endif

#endif /* SP_APP_H */
