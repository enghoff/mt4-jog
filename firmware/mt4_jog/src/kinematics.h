#ifndef MT4_KINEMATICS_H
#define MT4_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MT4_NUM_JOINTS 4
#define MT4_CJ_MASTER 10000L

/* +1 if positive step count increases firmware joint angle, -1 if inverted.
 * J3 confirmed inverted 2026-07-06: a +299-step probe raised the forearm
 * tip instead of lowering it as the old -1.0 sign predicted (photo-confirmed). */
#define MT4_J1_STEP_SIGN 1.0f
#define MT4_J2_STEP_SIGN (-1.0f)
#define MT4_J3_STEP_SIGN 1.0f
#define MT4_J4_STEP_SIGN 1.0f

/* Model angles at the homed pose (step counters = 0): base centered, upper
 * arm vertical (90 deg from horizontal), forearm horizontal (absolute angle;
 * the parallel linkage keeps it independent of J2). */
#define MT4_HOME_J1_DEG 0.0f
#define MT4_HOME_J2_DEG 90.0f
#define MT4_HOME_J3_DEG 0.0f
#define MT4_HOME_J4_DEG 0.0f

/* Default wrist-unwind gain (dq4 = -orient_gain * dq1); empirical, runtime
 * tunable via the `orient <gain>` serial command. */
#define MT4_ORIENT_GAIN_DEFAULT 1.0f

extern const float MT4_STEPS_PER_DEG[MT4_NUM_JOINTS];

typedef struct {
  float j1;
  float j2;
  float j3;
  float j4;
} JointAnglesDeg;

typedef struct {
  float x;
  float y;
  float z;
} Vec3;

typedef struct {
  int32_t j1;
  int32_t j2;
  int32_t j3;
  int32_t j4;
  int32_t master;
} CartesianRates;

void mt4_fk_tcp(const JointAnglesDeg *q, Vec3 *out);
bool mt4_cartesian_rates(const JointAnglesDeg *q, const Vec3 *dir_unit,
                         bool hold_orient, float orient_gain,
                         CartesianRates *out);

#ifdef __cplusplus
}
#endif

#endif
