#include "kinematics.h"

#include <math.h>
#include <stdio.h>

/* The MT4 is a parallel-link (palletizing) arm:
 *  - J2 sets the upper-arm absolute angle q2 (from horizontal).
 *  - J3 sets the forearm absolute angle q3 via the link rods, so the forearm
 *    does NOT rotate when J2 moves.
 *  - The head platform stays level: HEAD_OFFSET is horizontal (radially out),
 *    HEAD_HEIGHT drops the TCP below the wrist pivot.
 * Home pose q = (0, 90, 0): upper arm vertical, forearm horizontal — matches
 * the factory-reported TCP (230.000, 0, 255.570) exactly.
 */
static const float LINKAGE1 = 130.0f;      /* shoulder -> elbow */
static const float LINKAGE2 = 150.0f;      /* elbow -> wrist pivot */
static const float CENCER_OFFSET = 45.0f;  /* J1 axis -> shoulder, horizontal */
static const float CENCER_HEIGHT = 140.0f; /* shoulder pivot height */
static const float HEAD_OFFSET = 35.0f;    /* wrist pivot -> TCP, horizontal */
static const float HEAD_HEIGHT = 14.43f;   /* TCP below wrist pivot */

/* All four measured 2026-07-06 (J2-J4 with a phone clinometer against the
 * link; J1 by direct measurement of its yaw rotation), replacing the
 * factory-EEPROM-derived guesses -- J1/J2/J3 share a physical motor/gearbox
 * design (~35 steps/deg each). J3's own EEPROM setting was missing from the
 * dump entirely (the old 35.556 was borrowed from unrelated extra axes),
 * and J4's old value (852) was a wrong axis-letter assumption ("d" = J4). */
const float MT4_STEPS_PER_DEG[MT4_NUM_JOINTS] = {35.0f, 35.0f, 35.0f,
                                                 45.0f};
static const float J_STEP_SIGN[MT4_NUM_JOINTS] = {
    MT4_J1_STEP_SIGN, MT4_J2_STEP_SIGN, MT4_J3_STEP_SIGN, MT4_J4_STEP_SIGN};
static const float DLS_LAMBDA = 0.05f;

void mt4_fk_tcp(const JointAnglesDeg *q, Vec3 *out) {
  const float q1 = q->j1 * (float)(M_PI / 180.0);
  const float q2 = q->j2 * (float)(M_PI / 180.0);
  const float q3 = q->j3 * (float)(M_PI / 180.0);

  const float radial = CENCER_OFFSET + LINKAGE1 * cosf(q2) +
                       LINKAGE2 * cosf(q3) + HEAD_OFFSET;
  out->x = radial * cosf(q1);
  out->y = radial * sinf(q1);
  out->z = CENCER_HEIGHT + LINKAGE1 * sinf(q2) + LINKAGE2 * sinf(q3) -
           HEAD_HEIGHT;
}

static void jacobian_mm_per_deg(const JointAnglesDeg *q, float j[3][MT4_NUM_JOINTS]) {
  Vec3 p0;
  JointAnglesDeg trial = *q;
  mt4_fk_tcp(&trial, &p0);

  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    float *slot = &((float *)&trial.j1)[i];
    const float saved = *slot;
    *slot = saved + 0.1f;
    Vec3 p1;
    mt4_fk_tcp(&trial, &p1);
    *slot = saved;

    j[0][i] = (p1.x - p0.x) / 0.1f;
    j[1][i] = (p1.y - p0.y) / 0.1f;
    j[2][i] = (p1.z - p0.z) / 0.1f;
  }
}

static bool solve_dls3(const float j[3][3], const float v[3], float dq[3]) {
  float jjt[3][3];
  for (uint8_t r = 0; r < 3; ++r) {
    for (uint8_t c = 0; c < 3; ++c) {
      float sum = 0.0f;
      for (uint8_t k = 0; k < 3; ++k) {
        sum += j[r][k] * j[c][k];
      }
      jjt[r][c] = sum;
    }
  }

  const float lambda2 = DLS_LAMBDA * DLS_LAMBDA;
  for (uint8_t i = 0; i < 3; ++i) {
    jjt[i][i] += lambda2;
  }

  float m[3][4];
  for (uint8_t i = 0; i < 3; ++i) {
    for (uint8_t k = 0; k < 3; ++k) {
      m[i][k] = jjt[i][k];
    }
    m[i][3] = v[i];
  }

  for (uint8_t col = 0; col < 3; ++col) {
    uint8_t pivot = col;
    float best = fabsf(m[col][col]);
    for (uint8_t r = col + 1; r < 3; ++r) {
      const float val = fabsf(m[r][col]);
      if (val > best) {
        best = val;
        pivot = r;
      }
    }
    if (best < 1e-6f) {
      return false;
    }
    if (pivot != col) {
      for (uint8_t k = col; k < 4; ++k) {
        const float tmp = m[col][k];
        m[col][k] = m[pivot][k];
        m[pivot][k] = tmp;
      }
    }

    const float div = m[col][col];
    for (uint8_t k = col; k < 4; ++k) {
      m[col][k] /= div;
    }
    for (uint8_t r = 0; r < 3; ++r) {
      if (r == col) {
        continue;
      }
      const float factor = m[r][col];
      for (uint8_t k = col; k < 4; ++k) {
        m[r][k] -= factor * m[col][k];
      }
    }
  }

  const float y[3] = {m[0][3], m[1][3], m[2][3]};
  for (uint8_t c = 0; c < 3; ++c) {
    float sum = 0.0f;
    for (uint8_t r = 0; r < 3; ++r) {
      sum += j[r][c] * y[r];
    }
    dq[c] = sum;
  }
  return true;
}

bool mt4_cartesian_rates(const JointAnglesDeg *q, const Vec3 *dir_unit,
                         bool hold_orient, float orient_gain,
                         CartesianRates *out) {
  float j[3][MT4_NUM_JOINTS];
  jacobian_mm_per_deg(q, j);

  float a[3][3];
  for (uint8_t r = 0; r < 3; ++r) {
    for (uint8_t c = 0; c < 3; ++c) {
      a[r][c] = j[r][c];
    }
  }

  const float b[3] = {dir_unit->x, dir_unit->y, dir_unit->z};
  float dq[3];
  if (!solve_dls3(a, b, dq)) {
    return false;
  }

  /* dq is in model-angle space; wrist unwind counters base yaw before the
   * per-driver step signs are applied. orient_gain is empirical (real J1/J4
   * axis alignment and any mechanical wrist coupling are not modeled) and is
   * runtime-tunable via the `orient <gain>` serial command. */
  float dq4 = 0.0f;
  if (hold_orient && fabsf(dq[0]) > 1e-4f) {
    dq4 = -orient_gain * dq[0];
  }

  const float steps[4] = {
      J_STEP_SIGN[0] * dq[0] * MT4_STEPS_PER_DEG[0],
      J_STEP_SIGN[1] * dq[1] * MT4_STEPS_PER_DEG[1],
      J_STEP_SIGN[2] * dq[2] * MT4_STEPS_PER_DEG[2],
      J_STEP_SIGN[3] * dq4 * MT4_STEPS_PER_DEG[3]};

  /* Peak/master-scale spans all four joints. This used to be a problem when
   * J4's steps/deg (852, an axis-letter misassignment -- see kinematics.h)
   * was ~19x J1's, letting a modest orientation-hold correction dominate the
   * DDA timing budget and throttle the primary motion to a crawl. Now that
   * J4 is correctly calibrated (~45, close to J1's ~44), including it here
   * costs at most a few percent of speed and gives exact wrist-unwind
   * fidelity instead of clamping J4 short of the commanded orient_gain. */
  float peak = 0.0f;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    const float v = fabsf(steps[i]);
    if (v > peak) {
      peak = v;
    }
  }
  if (peak < 1e-6f) {
    return false;
  }

  const float scale = (float)MT4_CJ_MASTER / peak;
  out->j1 = (int32_t)lroundf(steps[0] * scale);
  out->j2 = (int32_t)lroundf(steps[1] * scale);
  out->j3 = (int32_t)lroundf(steps[2] * scale);
  out->j4 = (int32_t)lroundf(steps[3] * scale);
  out->master = MT4_CJ_MASTER;
  return true;
}
