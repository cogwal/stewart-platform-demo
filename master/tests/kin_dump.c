/* Reads poses (x y z roll pitch yaw per line) on stdin, prints the C IK strokes, the limited pose, and the
 * forward kinematics of the limited pose's strokes (ok flag + pose).
 * Used by tests/master/test_c_master.py to cross-check sp_kin.c against stewart/geometry.py. */
#include <stdio.h>

#include "sp_kin.h"

int main(void) {
    sp_pose_t p;
    while (scanf("%f %f %f %f %f %f", &p.x, &p.y, &p.z, &p.roll, &p.pitch, &p.yaw) == 6) {
        float s[6];
        sp_kin_strokes(&p, s);
        sp_pose_t l = sp_kin_limit(&p), f;
        float sl[6];
        sp_kin_strokes(&l, sl);
        int ok = sp_kin_forward(sl, NULL, &f);
        printf("%.5f %.5f %.5f %.5f %.5f %.5f | %.5f %.5f %.5f %.5f %.5f %.5f | %d %.5f %.5f %.5f %.5f %.5f %.5f\n",
               s[0], s[1], s[2], s[3], s[4], s[5], l.x, l.y, l.z, l.roll, l.pitch, l.yaw, ok, f.x, f.y, f.z, f.roll,
               f.pitch, f.yaw);
    }
    return 0;
}
