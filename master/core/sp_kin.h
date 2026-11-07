/* Stewart platform inverse kinematics and workspace limiting (single-precision float: the TC3xx FPU
 * is single precision). Same definitions as the Python reference stewart/geometry.py:
 *   pose = x, y, z [mm], roll, pitch, yaw [deg], relative to the neutral pose
 *   R = Rz(yaw) * Ry(pitch) * Rx(roll); stroke = leg length - closed length
 */
#ifndef SP_KIN_H
#define SP_KIN_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x, y, z;          /* mm */
    float roll, pitch, yaw; /* deg */
} sp_pose_t;

#define SP_POSE_AXES 6

/* Inverse kinematics: stroke of each leg in mm from the retracted end stop. */
void sp_kin_strokes(const sp_pose_t* pose, float strokes[6]);
/* All legs inside the soft limits minus the workspace margin. */
bool sp_kin_reachable(const sp_pose_t* pose);
/* Clamp to the configured axis limits, then scale toward neutral until reachable. */
sp_pose_t sp_kin_limit(const sp_pose_t* pose);
/* z offset of the level, centred pose with every leg at the given stroke. */
float sp_kin_level_z(float stroke_mm);

/* Forward kinematics (Newton on the IK residual, numeric Jacobian). guess: start pose (e.g. the previous result).
 * Returns false if it did not converge to 0.01 mm. */
bool sp_kin_forward(const float strokes[6], const sp_pose_t* guess, sp_pose_t* out);

float* sp_pose_axis(sp_pose_t* pose, int axis);

#ifdef __cplusplus
}
#endif

#endif /* SP_KIN_H */
