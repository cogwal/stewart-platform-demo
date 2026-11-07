/* Unit tests for the master core, motion and app layers (fake bus, fake time). */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fake_bus.h"

static int failures, checks;
#define CHECK(cond)                                                                     \
    do {                                                                                \
        checks++;                                                                       \
        if (!(cond)) {                                                                  \
            failures++;                                                                 \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                    \
        }                                                                               \
    } while (0)
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

static fake_t F;

static bool is_idle(fake_t* f) { return f->app.core.phase == SP_PH_IDLE; }
static bool is_run(fake_t* f) { return f->app.core.phase == SP_PH_RUN; }
static bool is_fault(fake_t* f) { return f->app.core.phase == SP_PH_FAULT; }
static bool is_stopped(fake_t* f) { return f->app.core.phase == SP_PH_STOPPED; }

static void test_json(void) {
    char s[16];
    float v, a[6];
    const char* j = "{\"type\": \"pose\", \"x\": 1.5, \"roll\":-2e1, \"mm\":[1, 2.5,3], \"source\":\"imu\"}";
    CHECK(sp_json_string(j, "type", s, sizeof s) == 0 && strcmp(s, "pose") == 0);
    CHECK(sp_json_string(j, "source", s, sizeof s) == 0 && strcmp(s, "imu") == 0);
    CHECK(sp_json_number(j, "x", &v) == 0 && v == 1.5f);
    CHECK(sp_json_number(j, "roll", &v) == 0 && v == -20.0f);
    CHECK(sp_json_number(j, "y", &v) != 0);
    CHECK(sp_json_numbers(j, "mm", a, 6) == 3 && a[1] == 2.5f && a[2] == 3.0f);
    CHECK(sp_json_number("{\"xx\":3,\"x\":4}", "x", &v) == 0 && v == 4.0f);
}

static void test_kinematics(void) {
    float s[6];
    sp_pose_t neutral = {0};
    sp_kin_strokes(&neutral, s);
    for (int i = 0; i < 6; i++) {
        NEAR(s[i], SP_NEUTRAL_STROKE_MM, 1e-3);
    }
    sp_pose_t up = {0, 0, 10, 0, 0, 0};
    sp_kin_strokes(&up, s);
    CHECK(s[0] > SP_NEUTRAL_STROKE_MM + 9.0f && s[0] < SP_NEUTRAL_STROKE_MM + 12.0f);
    sp_pose_t level = {0};
    level.z = sp_kin_level_z(SP_HOMING_PARK_MM);
    sp_kin_strokes(&level, s);
    NEAR(s[3], SP_HOMING_PARK_MM, 1e-3);
    sp_pose_t far = {0, 0, 0, 40, 40, 0};
    sp_pose_t lim = sp_kin_limit(&far);
    CHECK(lim.roll <= SP_LIMIT_ROLL_DEG && sp_kin_reachable(&lim));
    sp_pose_t corner = {SP_LIMIT_X_MM, SP_LIMIT_Y_MM, SP_LIMIT_Z_MM, SP_LIMIT_ROLL_DEG, SP_LIMIT_PITCH_DEG, SP_LIMIT_YAW_DEG};
    lim = sp_kin_limit(&corner);
    CHECK(sp_kin_reachable(&lim) && lim.x <= SP_LIMIT_X_MM && lim.yaw <= SP_LIMIT_YAW_DEG);
}

static void test_tables(void) {
    int node_flags = 0, master_flags = 0;
    for (unsigned i = 0; i < SP_STARTUP_SDO_COUNT; i++) {
        if (SP_STARTUP_SDO[i].flags & SP_SDO_ADD_NODE) node_flags++;
        if (SP_STARTUP_SDO[i].flags & SP_SDO_ADD_MASTER_HB) {
            master_flags++;
            CHECK(SP_STARTUP_SDO[i].index == 0x1016);
        }
    }
    CHECK(node_flags >= 10); /* every PDO COB-ID */
    CHECK(master_flags == 1);
    CHECK(sp_emcy_classify(0x3230, 0) == SP_EMCY_HOMING_STALL);
    CHECK(sp_emcy_classify(0x4310, 1) == SP_EMCY_WARN);
    CHECK(sp_emcy_classify(0x4310, 2) == SP_EMCY_STOP_ALL_REHOME);
    CHECK(sp_emcy_classify(0x1234, 0) == SP_EMCY_STOP_ALL);
}

static void test_joystick(void) {
    NEAR(sp_joystick_shape(0.0f), 0.0, 1e-6);
    NEAR(sp_joystick_shape(SP_JOY_DEADBAND), 0.0, 1e-6);
    NEAR(sp_joystick_shape(1.0f), 1.0, 1e-6);
    NEAR(sp_joystick_shape(-1.0f), -1.0, 1e-6);
    CHECK(sp_joystick_shape(0.5f) < 0.5f && sp_joystick_shape(0.5f) > 0.0f);
}

static void boot_and_home(uint8_t legs) {
    fake_init(&F, legs, 37.0, true);
    CHECK(fake_run_until(&F, 3000, is_idle) || F.app.core.phase == SP_PH_HOMING);
    for (uint8_t i = 0; i < legs; i++) {
        CHECK(F.d[i].sdo_writes >= SP_STARTUP_SDO_COUNT);
        CHECK(F.d[i].nmt == SP_NMTS_OPERATIONAL);
    }
    CHECK(fake_run_until(&F, 40000, is_run));
}

static void test_boot_home_follow(void) {
    boot_and_home(6);
    CHECK(F.app.core.homed);
    for (int i = 0; i < 6; i++) {
        NEAR(F.d[i].truth_mm, SP_HOMING_PARK_MM, 0.05); /* zero = the real end stop */
        NEAR(F.d[i].counts, SP_HOMING_PARK_MM * SP_COUNTS_PER_MM, 2);
        CHECK(F.d[i].current == SP_CURRENT_RUN);
        CHECK(F.d[i].mode == SP_MODE_CYCLIC_SYNC_POSITION);
    }
    /* Pose source, neutral: legs rise to the neutral stroke, level. */
    fake_run(&F, 8000);
    for (int i = 0; i < 6; i++) {
        NEAR(F.d[i].truth_mm, SP_NEUTRAL_STROKE_MM, 0.05);
    }
    /* Tilt: legs follow the IK of the commanded pose. */
    CHECK(sp_app_command(&F.app, "{\"type\":\"pose\",\"roll\":5,\"pitch\":-3,\"z\":8}") == 0);
    fake_run(&F, 6000);
    float s[6];
    sp_pose_t p = {0, 0, 8, 5, -3, 0};
    sp_kin_strokes(&p, s);
    for (int i = 0; i < 6; i++) {
        NEAR(F.d[i].truth_mm, s[i], 0.05);
    }
    /* Leg speed never above the csp limit: measured over one SYNC. */
    CHECK(sp_app_command(&F.app, "{\"type\":\"pose\",\"roll\":-10,\"pitch\":10}") == 0);
    double worst = 0;
    for (int k = 0; k < 300; k++) {
        double before[6];
        for (int i = 0; i < 6; i++) before[i] = F.d[i].truth_mm;
        fake_run(&F, SP_SYNC_PERIOD_MS);
        for (int i = 0; i < 6; i++) {
            double v = fabs(F.d[i].truth_mm - before[i]) / (SP_SYNC_PERIOD_MS / 1000.0);
            if (v > worst) worst = v;
        }
    }
    CHECK(worst <= SP_CSP_MAX_VELOCITY_MM_S * 1.01);
    CHECK(worst > SP_CSP_MAX_VELOCITY_MM_S * 0.5);
}

static void test_stop_resume(void) {
    boot_and_home(6);
    fake_run(&F, 2000);
    CHECK(sp_app_command(&F.app, "{\"type\":\"stop\"}") == 0);
    fake_run(&F, 50);
    CHECK(is_stopped(&F) && F.app.core.homed);
    for (int i = 0; i < 6; i++) CHECK(F.d[i].state == FD_SOD);
    CHECK(sp_app_command(&F.app, "{\"type\":\"resume\"}") == 0);
    CHECK(fake_run_until(&F, 2000, is_run));
    fake_run(&F, 100);
    CHECK(sp_core_all_legs_in_csp(&F.app.core));
    fake_run(&F, 8000);
    for (int i = 0; i < 6; i++) NEAR(F.d[i].truth_mm, SP_NEUTRAL_STROKE_MM, 0.05);
}

static void test_leg_source_single_leg(void) {
    boot_and_home(1);
    CHECK(sp_app_command(&F.app, "{\"type\":\"setpoint\",\"mm\":20}") == 0);
    fake_run(&F, 3000);
    NEAR(F.d[0].truth_mm, 20.0, 0.01);
    CHECK(sp_app_command(&F.app, "{\"type\":\"setpoint\",\"mm\":500}") == 0);
    fake_run(&F, 12000);
    NEAR(F.d[0].truth_mm, SP_SOFT_MAX_MM, 0.01);
}

static void test_reboot_loses_homing(void) {
    boot_and_home(6);
    fake_power_cycle(&F, 2);
    fake_run(&F, 20);
    CHECK(is_fault(&F) && !F.app.core.homed);
    CHECK(strstr(F.app.core.reason, "rebooted") != NULL);
    /* Home again: the rebooted drive is reconfigured, then all legs home. */
    CHECK(sp_app_command(&F.app, "{\"type\":\"home\"}") == 0);
    CHECK(fake_run_until(&F, 40000, is_run));
    NEAR(F.d[2].truth_mm, SP_HOMING_PARK_MM, 0.05);
}

static void test_heartbeat_loss(void) {
    boot_and_home(6);
    fake_run(&F, 1000);
    F.d[4].hb_alive = false;
    fake_run(&F, 20);
    CHECK(is_stopped(&F) && F.app.core.homed);
    CHECK(F.app.core.legs[4].boot == SP_BOOT_LOST);
    CHECK(!sp_core_cmd_resume(&F.app.core)); /* not while a drive is missing */
    F.d[4].hb_alive = true;
    fake_run(&F, 20);
    CHECK(F.app.core.legs[4].boot == SP_BOOT_READY);
    CHECK(sp_core_cmd_resume(&F.app.core));
    CHECK(fake_run_until(&F, 2000, is_run));
}

static void test_emcy_classes(void) {
    boot_and_home(6);
    sp_core_on_emcy(&F.app.core, 1, 0x4310, 0, 1); /* overtemp pre-warning: info only */
    CHECK(is_run(&F));
    sp_core_on_emcy(&F.app.core, 3, 0x8130, 0, 0); /* drive lost master heartbeat: stop, keep homing */
    CHECK(is_stopped(&F) && F.app.core.homed);
    CHECK(sp_core_cmd_resume(&F.app.core));
    CHECK(fake_run_until(&F, 2000, is_run));
    sp_core_on_emcy(&F.app.core, 5, 0x5441, 0, 0); /* ENN: stop and lose homing */
    CHECK(is_fault(&F) && !F.app.core.homed);
    CHECK(!sp_core_cmd_resume(&F.app.core));
}

static void test_drive_nmt_stopped_resume(void) {
    boot_and_home(6);
    /* Drive lost the master heartbeat: NMT stopped + fault + EMCY 8130 (as the real firmware). */
    F.d[1].nmt = SP_NMTS_STOPPED;
    F.d[1].state = FD_FAULT;
    F.d[1].emcy_pending = 0x8130;
    fake_run(&F, 20);
    CHECK(is_stopped(&F) && F.app.core.homed);
    CHECK(sp_core_cmd_resume(&F.app.core));
    CHECK(fake_run_until(&F, 3000, is_run));
    fake_run(&F, 50);
    CHECK(F.d[1].nmt == SP_NMTS_OPERATIONAL && F.d[1].state == FD_OE);
}

static void test_homing_by_load_and_timeout(void) {
    /* stallGuard stop disabled: the end stop is found by polling 2101h. */
    fake_init(&F, 2, 30.0, true);
    F.d[0].no_stall_detect = true;
    F.d[1].no_stall_detect = true;
    CHECK(fake_run_until(&F, 45000, is_run));
    NEAR(F.d[0].truth_mm, SP_HOMING_PARK_MM, 0.05);
    NEAR(F.d[1].truth_mm, SP_HOMING_PARK_MM, 0.05);
}

static void test_missing_drive_retries(void) {
    fake_init(&F, 6, 37.0, true);
    F.d[5].powered = false;
    fake_run(&F, SP_TIMEOUT_BOOTUP_MS + 500);
    CHECK(F.app.core.phase == SP_PH_BOOT);
    CHECK(F.app.core.legs[5].boot == SP_BOOT_ERROR || F.app.core.legs[5].boot == SP_BOOT_RESET ||
          F.app.core.legs[5].boot == SP_BOOT_WAIT);
    CHECK(strstr(F.app.core.reason, "no boot-up") != NULL);
    F.d[5].powered = true; /* plugged in: the next retry finds it */
    CHECK(fake_run_until(&F, 50000, is_run));
}

static void test_sdo_hang_guard(void) {
    fake_init(&F, 1, 37.0, false);
    F.sdo_hang = true;
    fake_run(&F, 4 * SP_TIMEOUT_SDO_MS + 200);
    CHECK(F.app.core.legs[0].boot == SP_BOOT_ERROR);
    F.sdo_hang = false;
    F.sdo_active = false;
    CHECK(fake_run_until(&F, 10000, is_idle));
}

static void test_status_json(void) {
    char buf[4096];
    boot_and_home(6);
    int n = sp_app_status_json(&F.app, buf, sizeof buf);
    CHECK(n > 100 && n < (int)sizeof buf);
    CHECK(strstr(buf, "\"phase\":\"run\"") != NULL);
    CHECK(strstr(buf, "\"legs\":[{\"node\":1") != NULL);
    CHECK(buf[n - 1] == '}');
    char small[64];
    n = sp_app_status_json(&F.app, small, sizeof small);
    CHECK(n <= (int)sizeof small);
}

static void test_demo_and_imu(void) {
    boot_and_home(6);
    CHECK(sp_app_command(&F.app, "{\"type\":\"source\",\"source\":\"imu\"}") == 0);
    CHECK(sp_app_command(&F.app, "{\"type\":\"imu\",\"roll\":40,\"pitch\":-4}") == 0);
    fake_run(&F, 8000);
    NEAR(F.app.motion.target.roll, SP_IMU_CLAMP_DEG > SP_LIMIT_ROLL_DEG ? SP_LIMIT_ROLL_DEG : SP_IMU_CLAMP_DEG, 1.0);
    CHECK(sp_kin_reachable(&F.app.motion.pose));
    CHECK(sp_app_command(&F.app, "{\"type\":\"source\",\"source\":\"demo\"}") == 0);
    fake_run(&F, 3000);
    CHECK(F.app.motion.demo_active);
    CHECK(sp_app_command(&F.app, "{\"type\":\"source\",\"source\":\"bogus\"}") != 0);
    CHECK(sp_app_command(&F.app, "{\"type\":\"joystick\",\"x\":1,\"y\":0}") == 0);
}

int main(void) {
    struct {
        const char* name;
        void (*fn)(void);
    } tests[] = {
        {"json", test_json},
        {"kinematics", test_kinematics},
        {"tables", test_tables},
        {"joystick", test_joystick},
        {"boot_home_follow", test_boot_home_follow},
        {"stop_resume", test_stop_resume},
        {"leg_source_single_leg", test_leg_source_single_leg},
        {"reboot_loses_homing", test_reboot_loses_homing},
        {"heartbeat_loss", test_heartbeat_loss},
        {"emcy_classes", test_emcy_classes},
        {"drive_nmt_stopped_resume", test_drive_nmt_stopped_resume},
        {"homing_by_load", test_homing_by_load_and_timeout},
        {"missing_drive_retries", test_missing_drive_retries},
        {"sdo_hang_guard", test_sdo_hang_guard},
        {"status_json", test_status_json},
        {"demo_and_imu", test_demo_and_imu},
    };
    for (unsigned i = 0; i < sizeof tests / sizeof tests[0]; i++) {
        int before = failures;
        tests[i].fn();
        printf("%s %s\n", failures == before ? "ok  " : "FAIL", tests[i].name);
        if (failures != before) {
            printf("    phase=%s reason=%s\n", sp_phase_name(F.app.core.phase), F.app.core.reason);
        }
    }
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
