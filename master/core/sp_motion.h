/* Operator input -> leg targets. Sources: a pose (web sliders / commissioning), the ECU's analog
 * joystick, an IMU, a built-in demo motion, or direct leg positions (single-leg tests, bench).
 *
 * Per SYNC: target pose from the source -> axis limits + workspace -> smoothing in pose space
 * (velocity/acceleration per axis) -> step scaled so no leg exceeds the csp speed limit -> IK.
 * Portable C99, time passed in.
 */
#ifndef SP_MOTION_H
#define SP_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "sp_kin.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { SP_SRC_POSE, SP_SRC_JOYSTICK, SP_SRC_IMU, SP_SRC_DEMO, SP_SRC_LEGS, SP_SRC_COUNT } sp_source_t;

typedef struct {
    sp_source_t source;
    sp_pose_t pose_cmd;  /* SP_SRC_POSE */
    float joy[2];        /* SP_SRC_JOYSTICK, -1..1 */
    float imu[3];        /* SP_SRC_IMU roll, pitch, yaw [deg] */
    float legs_cmd[6];   /* SP_SRC_LEGS [mm] */
    float demo_t;        /* s */
    float idle_s;        /* time since the last operator input */
    bool demo_active;    /* demo running (selected, or idle timeout) */

    sp_pose_t target;    /* limited target pose */
    sp_pose_t pose;      /* smoothed pose: what the legs are commanded to */
    float vel[SP_POSE_AXES];
    float legs[6];       /* output [mm] */
} sp_motion_t;

void sp_motion_init(sp_motion_t* m);
void sp_motion_set_source(sp_motion_t* m, sp_source_t source);
void sp_motion_set_pose(sp_motion_t* m, const sp_pose_t* pose);
void sp_motion_set_joystick(sp_motion_t* m, float x, float y);
void sp_motion_set_imu(sp_motion_t* m, float roll, float pitch, float yaw);
void sp_motion_set_legs(sp_motion_t* m, const float* mm, uint8_t count);
/* Restart smoothing from a level pose with every leg at stroke_mm (after homing). */
void sp_motion_reset_level(sp_motion_t* m, float stroke_mm);
/* Advance by dt seconds; writes 6 leg targets [mm]. */
void sp_motion_step(sp_motion_t* m, float dt, float legs_out[6]);

float sp_joystick_shape(float v);
const char* sp_source_name(sp_source_t s);
bool sp_source_parse(const char* name, sp_source_t* out);

#ifdef __cplusplus
}
#endif

#endif /* SP_MOTION_H */
