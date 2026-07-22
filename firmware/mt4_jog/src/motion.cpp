#include "motion.h"

#include <math.h>

#include "dda.h"
#include "gripper.h"
#include "pins.h"

bool cart_orient_hold = true;
bool mt4_homed = false;

int32_t joint_soft_min[MT4_NUM_JOINTS] = {
    MT4_JOINT_SOFT_MIN_DEFAULT[0], MT4_JOINT_SOFT_MIN_DEFAULT[1],
    MT4_JOINT_SOFT_MIN_DEFAULT[2], MT4_JOINT_SOFT_MIN_DEFAULT[3]};
int32_t joint_soft_max[MT4_NUM_JOINTS] = {
    MT4_JOINT_SOFT_MAX_DEFAULT[0], MT4_JOINT_SOFT_MAX_DEFAULT[1],
    MT4_JOINT_SOFT_MAX_DEFAULT[2], MT4_JOINT_SOFT_MAX_DEFAULT[3]};
float mt4_ground_z_mm = MT4_GROUND_Z_MM;

static bool cart_jog_mode = false;
static Vec3 cart_dir_active = {0.0f, 0.0f, 0.0f};
static bool cart_dir_active_valid = false;
static int8_t cart_j4_roll = 0;
static unsigned long cart_refresh_ms = 0;

static void mp_path_cancel();
static void report_move_done();

static JointAnglesDeg angles_from_joint_steps(const long steps[MT4_NUM_JOINTS]) {
  JointAnglesDeg q;
  q.j1 = MT4_HOME_J1_DEG + MT4_J1_STEP_SIGN *
                               static_cast<float>(steps[0]) /
                               MT4_STEPS_PER_DEG[0];
  q.j2 = MT4_HOME_J2_DEG + MT4_J2_STEP_SIGN *
                               static_cast<float>(steps[1]) /
                               MT4_STEPS_PER_DEG[1];
  q.j3 = MT4_HOME_J3_DEG + MT4_J3_STEP_SIGN *
                               static_cast<float>(steps[2]) /
                               MT4_STEPS_PER_DEG[2];
  q.j4 = MT4_HOME_J4_DEG + MT4_J4_STEP_SIGN *
                               static_cast<float>(steps[3]) /
                               MT4_STEPS_PER_DEG[3];
  return q;
}

static JointAnglesDeg angles_from_steps() {
  long steps[MT4_NUM_JOINTS];
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    steps[i] = joint_steps[i];
  }
  return angles_from_joint_steps(steps);
}

void motion_init() {
  dda_init();
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    joint_soft_min[i] = MT4_JOINT_SOFT_MIN_DEFAULT[i];
    joint_soft_max[i] = MT4_JOINT_SOFT_MAX_DEFAULT[i];
  }
  mt4_ground_z_mm = MT4_GROUND_Z_MM;
}

void motion_apply_home_soft_limits(uint16_t j1_center, uint16_t j2_pull,
                                   uint16_t j3_pull) {
  // Pull-offs only set post-home park counters (see do_home); J2/J3 soft
  // limits are limit-referenced and come from the envelope defaults.
  (void)j2_pull;
  (void)j3_pull;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    joint_soft_min[i] = MT4_JOINT_SOFT_MIN_DEFAULT[i];
    joint_soft_max[i] = MT4_JOINT_SOFT_MAX_DEFAULT[i];
  }
  // J1: home seek uses DIR high (negative counts); park at steps=0 after
  // centering, so the switch sits at -j1_center.
  // J2: counters are limit-referenced (steps=0 at the switch).
  joint_soft_min[0] = -static_cast<int32_t>(j1_center);
  joint_soft_min[1] = 0;

  Serial.print(F("home limits J1="));
  Serial.print(joint_soft_min[0]);
  Serial.print(F(".."));
  Serial.print(joint_soft_max[0]);
  Serial.print(F(" J2="));
  Serial.print(joint_soft_min[1]);
  Serial.print(F(".."));
  Serial.print(joint_soft_max[1]);
  Serial.print(F(" J3="));
  Serial.print(joint_soft_min[2]);
  Serial.print(F(".."));
  Serial.print(joint_soft_max[2]);
  Serial.print(F(" J4="));
  Serial.print(joint_soft_min[3]);
  Serial.print(F(".."));
  Serial.print(joint_soft_max[3]);
  Serial.print(F(" J2+J3="));
  Serial.print(MT4_J2_J3_SUM_MIN);
  Serial.print(F(".."));
  Serial.print(MT4_J2_J3_SUM_MAX);
  Serial.print(F(" ground_z="));
  Serial.println(mt4_ground_z_mm, 1);
}

bool motion_joints_within_soft_limits(const long steps[MT4_NUM_JOINTS]) {
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (steps[i] < joint_soft_min[i] || steps[i] > joint_soft_max[i]) {
      return false;
    }
  }
  const int32_t sum23 = steps[1] + steps[2];
  if (sum23 > MT4_J2_J3_SUM_MAX || sum23 < MT4_J2_J3_SUM_MIN) {
    return false;
  }
  return true;
}

bool motion_step_allowed(uint8_t joint, bool positive) {
  if (joint >= MT4_NUM_JOINTS) {
    return false;
  }
  const int32_t next = joint_steps[joint] + (positive ? 1 : -1);
  if (next < joint_soft_min[joint] || next > joint_soft_max[joint]) {
    return false;
  }
  /* Coupled extension/fold limit on J2+J3 step sum. */
  if (joint == 1 || joint == 2) {
    const int32_t s2 = (joint == 1) ? next : joint_steps[1];
    const int32_t s3 = (joint == 2) ? next : joint_steps[2];
    const int32_t sum23 = s2 + s3;
    if (sum23 > MT4_J2_J3_SUM_MAX || sum23 < MT4_J2_J3_SUM_MIN) {
      return false;
    }
  }
  return true;
}

void reset_joint_steps() {
  dda_reset();
  cart_jog_mode = false;
  cart_dir_active_valid = false;
}

void motion_set_joint_steps(const long steps[MT4_NUM_JOINTS]) {
  dda_set_joint_steps(steps);
  Serial.print(F("ok "));
  print_joint_pos();
}

void stop_jog() {
  dda_stop();
}

void motion_cancel_move() {
  move_mode = false;
  move_done_pending = false;
  mp_path_cancel();
}

void print_joint_pos() {
  Serial.print(F("pos J1="));
  Serial.print(joint_steps[0]);
  Serial.print(F(" J2="));
  Serial.print(joint_steps[1]);
  Serial.print(F(" J3="));
  Serial.print(joint_steps[2]);
  Serial.print(F(" J4="));
  Serial.println(joint_steps[3]);

  // Derived absolute state for clients (e.g. the `mp` prompt script) to use
  // as move defaults -- same frame/units `mp` accepts (x/y/z mm, origin at
  // the base under J1's pivot; j4 deg world-frame yaw; grip S; speed us).
  const JointAnglesDeg q = angles_from_steps();
  Vec3 tcp;
  mt4_fk_tcp(&q, &tcp);
  Serial.print(F("tcp x="));
  Serial.print(tcp.x, 1);
  Serial.print(F(" y="));
  Serial.print(tcp.y, 1);
  Serial.print(F(" z="));
  Serial.print(tcp.z, 1);
  Serial.print(F(" j4="));
  Serial.print(mt4_ws_j4_deg(&q), 1);
  Serial.print(F(" grip="));
  Serial.print(gripper_s);
  Serial.print(F(" speed="));
  Serial.println(dda_get_speed_us());
}

// F()'s PSTR() expands to a GCC statement-expression, which can't be used
// as a static initializer at file scope -- so track which command is
// pending with a plain bool instead of caching the F() string itself.
static bool move_done_is_mp = false;

/* One piece of an `mp` path's XY track: a straight line or an arc along the
 * keep-out cylinder (radius MT4_KEEPOUT_RADIUS_MM, centered on the J1
 * axis). Lines: (a,b) -> (c,d). Arcs: start angle a, signed sweep b. */
typedef struct {
  uint8_t kind;  // 0 = line, 1 = arc
  float a, b, c, d;
  float len;
} MpPiece;

/* `mp` path: the XY track is a piecewise line/arc route around the keep-out
 * cylinder (a single straight line when the direct path clears it); Z and
 * J4 interpolate linearly over total path length. J4 is either held fixed
 * in world space (when the commanded J4 matches the pose at move start) or
 * interpolated linearly. */
static struct {
  bool active;
  bool hold_ws_orient;
  uint16_t next_seg;
  uint16_t num_segments;
  uint8_t num_pieces;
  float total_xy_len;
  float sz, ez;
  float sj4_ws, ej4_ws;
  MpPiece pieces[MP_MAX_PIECES];
} mp_path;

static bool mp_drivers_ready = false;

static void mp_path_cancel() {
  mp_path.active = false;
  mp_path.hold_ws_orient = false;
  mp_path.next_seg = 0;
  mp_path.num_segments = 0;
  mp_path.num_pieces = 0;
  mp_drivers_ready = false;
}

static float wrap_2pi_f(float a) {
  a = fmodf(a, 2.0f * (float)M_PI);
  return (a < 0.0f) ? a + 2.0f * (float)M_PI : a;
}

/* Closest approach of the XY segment (sx,sy)->(ex,ey) to the J1 axis. */
static float seg_dist_origin(float sx, float sy, float ex, float ey) {
  const float dx = ex - sx;
  const float dy = ey - sy;
  const float l2 = dx * dx + dy * dy;
  if (l2 < 1e-9f) {
    return sqrtf(sx * sx + sy * sy);
  }
  float t = -(sx * dx + sy * dy) / l2;
  if (t < 0.0f) {
    t = 0.0f;
  } else if (t > 1.0f) {
    t = 1.0f;
  }
  const float px = sx + t * dx;
  const float py = sy + t * dy;
  return sqrtf(px * px + py * py);
}

static float mp_piece_line(MpPiece *p, float ax, float ay, float bx, float by) {
  p->kind = 0;
  p->a = ax;
  p->b = ay;
  p->c = bx;
  p->d = by;
  p->len = sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
  return p->len;
}

/* Route the XY track from S to E around the keep-out cylinder: straight
 * when the direct segment clears it; otherwise entry tangent -> shortest
 * arc along the cylinder -> exit tangent. A start inside the cylinder
 * (reachable via joint-space moves / homing) gets a radial escape piece
 * first. The 0.25mm tolerance stops borderline float noise from forcing
 * detours, so the effective guarantee is R - 0.25mm. */
static uint8_t plan_keepout_path(float sx, float sy, float ex, float ey,
                                 MpPiece *out, float *total_len) {
  const float R = MT4_KEEPOUT_RADIUS_MM;
  uint8_t n = 0;
  float len = 0.0f;
  float cx = sx;
  float cy = sy;

  const float r_s = sqrtf(sx * sx + sy * sy);
  if (r_s < R - 0.25f) {
    float ux;
    float uy;
    if (r_s < 1e-6f) {
      const float r_e = sqrtf(ex * ex + ey * ey);
      ux = (r_e > 1e-6f) ? ex / r_e : 1.0f;
      uy = (r_e > 1e-6f) ? ey / r_e : 0.0f;
    } else {
      ux = sx / r_s;
      uy = sy / r_s;
    }
    len += mp_piece_line(&out[n++], cx, cy, ux * R, uy * R);
    cx = ux * R;
    cy = uy * R;
  }

  if (seg_dist_origin(cx, cy, ex, ey) >= R - 0.25f) {
    len += mp_piece_line(&out[n++], cx, cy, ex, ey);
    *total_len = len;
    return n;
  }

  const float d_s = sqrtf(cx * cx + cy * cy);
  const float d_e = sqrtf(ex * ex + ey * ey);
  const float th_s = atan2f(cy, cx);
  const float th_e = atan2f(ey, ex);
  float cs = R / d_s;
  float ce = R / d_e;
  if (cs > 1.0f) cs = 1.0f;
  if (ce > 1.0f) ce = 1.0f;
  const float ph_s = acosf(cs);
  const float ph_e = acosf(ce);

  /* Wrap counterclockwise: leave S touching at th_s+ph_s, rejoin toward E
   * at th_e-ph_e; clockwise mirrors. Tangent lengths are equal either way,
   * so the shorter arc decides the side. */
  const float a1 = th_s + ph_s;
  const float a2 = th_e - ph_e;
  const float d_ccw = wrap_2pi_f(a2 - a1);
  const float b1 = th_s - ph_s;
  const float b2 = th_e + ph_e;
  const float d_cw = wrap_2pi_f(b1 - b2);

  const float t1a = (d_ccw <= d_cw) ? a1 : b1;
  const float t2a = (d_ccw <= d_cw) ? a2 : b2;
  const float sweep = (d_ccw <= d_cw) ? d_ccw : -d_cw;

  len += mp_piece_line(&out[n++], cx, cy, R * cosf(t1a), R * sinf(t1a));
  out[n].kind = 1;
  out[n].a = t1a;
  out[n].b = sweep;
  out[n].c = 0.0f;
  out[n].d = 0.0f;
  out[n].len = R * fabsf(sweep);
  len += out[n].len;
  ++n;
  len += mp_piece_line(&out[n++], R * cosf(t2a), R * sinf(t2a), ex, ey);
  *total_len = len;
  return n;
}

/* XY position at arc length s along the planned pieces. */
static void mp_path_xy_at(float s, float *wx, float *wy) {
  for (uint8_t i = 0; i < mp_path.num_pieces; ++i) {
    const MpPiece *p = &mp_path.pieces[i];
    const bool last = (i == mp_path.num_pieces - 1);
    if (s <= p->len || last) {
      float u = (p->len < 1e-6f) ? 0.0f : s / p->len;
      if (u > 1.0f) {
        u = 1.0f;
      } else if (u < 0.0f) {
        u = 0.0f;
      }
      if (p->kind == 0) {
        *wx = p->a + u * (p->c - p->a);
        *wy = p->b + u * (p->d - p->b);
      } else {
        const float ang = p->a + u * p->b;
        *wx = MT4_KEEPOUT_RADIUS_MM * cosf(ang);
        *wy = MT4_KEEPOUT_RADIUS_MM * sinf(ang);
      }
      return;
    }
    s -= p->len;
  }
}

static void angles_to_joint_steps(const JointAnglesDeg *q,
                                  long out[MT4_NUM_JOINTS]) {
  out[0] = lroundf((q->j1 - MT4_HOME_J1_DEG) * MT4_STEPS_PER_DEG[0] *
                   MT4_J1_STEP_SIGN);
  out[1] = lroundf((q->j2 - MT4_HOME_J2_DEG) * MT4_STEPS_PER_DEG[1] *
                   MT4_J2_STEP_SIGN);
  out[2] = lroundf((q->j3 - MT4_HOME_J3_DEG) * MT4_STEPS_PER_DEG[2] *
                   MT4_J3_STEP_SIGN);
  out[3] = lroundf((q->j4 - MT4_HOME_J4_DEG) * MT4_STEPS_PER_DEG[3] *
                   MT4_J4_STEP_SIGN);
}

void motion_zero_j4_world() {
  // world_j4 = joint_j4 + j1; set joint_j4 = -j1 so world reports 0 at this
  // pose without moving. Soft limits stay centered on the new step-0, so the
  // usable wrist window is re-homed around the operator's alignment.
  stop_jog();
  motion_cancel_move();
  const JointAnglesDeg q = angles_from_steps();
  JointAnglesDeg target = q;
  target.j4 = -q.j1;
  long steps[MT4_NUM_JOINTS];
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    steps[i] = joint_steps[i];
  }
  long computed[MT4_NUM_JOINTS];
  angles_to_joint_steps(&target, computed);
  steps[3] = computed[3];
  dda_set_joint_steps(steps);
  Serial.print(F("ok j4zero "));
  print_joint_pos();
}

static bool joint_steps_to_deltas(const long current[MT4_NUM_JOINTS],
                                  const JointAnglesDeg *target,
                                  int32_t deltas[MT4_NUM_JOINTS],
                                  int32_t *out_master) {
  long target_steps[MT4_NUM_JOINTS];
  angles_to_joint_steps(target, target_steps);

  int32_t master = 0;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    const long d = target_steps[i] - current[i];
    if (d > MOVE_MAX_STEPS || d < -MOVE_MAX_STEPS) {
      return false;
    }
    deltas[i] = static_cast<int32_t>(d);
    const int32_t mag = deltas[i] < 0 ? -deltas[i] : deltas[i];
    if (mag > master) {
      master = mag;
    }
  }
  *out_master = master;
  return true;
}

static bool joint_angles_to_deltas(const JointAnglesDeg *target,
                                   int32_t deltas[MT4_NUM_JOINTS],
                                   int32_t *out_master) {
  long current[MT4_NUM_JOINTS];
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    current[i] = joint_steps[i];
  }
  return joint_steps_to_deltas(current, target, deltas, out_master);
}

/* Joint target at the end of Cartesian `mp` segment seg (1..num_segments). */
static bool mp_segment_target(uint16_t seg, const JointAnglesDeg *near,
                              JointAnglesDeg *out) {
  const float t =
      static_cast<float>(seg) / static_cast<float>(mp_path.num_segments);
  float wx;
  float wy;
  mp_path_xy_at(t * mp_path.total_xy_len, &wx, &wy);
  const float wz = mp_path.sz + t * (mp_path.ez - mp_path.sz);
  const float wj4_ws = mp_path.hold_ws_orient
                           ? mp_path.sj4_ws
                           : mp_path.sj4_ws + t * (mp_path.ej4_ws - mp_path.sj4_ws);

  if (!mt4_ik_position(near, wx, wy, wz, 0.0f, out)) {
    return false;
  }
  out->j4 = wj4_ws - out->j1;
  return true;
}

/* Sum per-segment master ticks along the planned Cartesian path so the
 * accel/decel ramp tracks detours and segmentation, not a straight
 * joint-space chord to the final pose. */
static int32_t mp_estimate_path_ticks(void) {
  long sim[MT4_NUM_JOINTS];
  const JointAnglesDeg q0 = angles_from_steps();
  angles_to_joint_steps(&q0, sim);

  int32_t total = 0;
  for (uint16_t seg = 1; seg <= mp_path.num_segments; ++seg) {
    const JointAnglesDeg near = angles_from_joint_steps(sim);
    JointAnglesDeg target;
    if (!mp_segment_target(seg, &near, &target)) {
      return 0;
    }
    int32_t deltas[MT4_NUM_JOINTS];
    int32_t master = 0;
    if (!joint_steps_to_deltas(sim, &target, deltas, &master)) {
      return 0;
    }
    total += master;
    for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
      sim[i] += deltas[i];
    }
  }
  return total;
}

/* Arm one Cartesian-linear `mp` segment (seg is 1..num_segments). Returns
 * false on IK/delta failure. When the segment needs no joint motion, returns
 * true with move_mode left false so the caller can chain immediately. */
static bool mp_execute_segment(uint16_t seg) {
  const JointAnglesDeg near = angles_from_steps();
  JointAnglesDeg target;
  if (!mp_segment_target(seg, &near, &target)) {
    return false;
  }
  {
    long target_steps[MT4_NUM_JOINTS];
    angles_to_joint_steps(&target, target_steps);
    if (!motion_joints_within_soft_limits(target_steps)) {
      return false;
    }
  }

  int32_t deltas[MT4_NUM_JOINTS];
  int32_t master = 0;
  if (!joint_angles_to_deltas(&target, deltas, &master)) {
    return false;
  }
  if (master == 0) {
    return true;
  }

  if (!mp_drivers_ready) {
    apply_all(PIN_FLOAT);
    set_enable(true);
    delay(ENABLE_SETTLE_MS);
    mp_drivers_ready = true;
  }
  dda_arm(master, deltas, true);
  dda_engage();
  return true;
}

/* Run pending `mp` segments until one needs async stepping or the path ends.
 * On success with async motion in flight, leaves mp_path.active true. On
 * completion, clears mp_path and emits "mp done pos ...". */
static bool mp_continue_path() {
  while (mp_path.active && mp_path.next_seg <= mp_path.num_segments) {
    if (!mp_execute_segment(mp_path.next_seg)) {
      mp_path_cancel();
      Serial.println(F("err mp segment"));
      return false;
    }
    ++mp_path.next_seg;
    if (move_mode) {
      return true;
    }
  }
  mp_path_cancel();
  report_move_done();
  return true;
}

static void report_move_done() {
  Serial.print(move_done_is_mp ? F("mp") : F("m"));
  Serial.print(F(" done "));
  print_joint_pos();
}

void print_status() {
  Serial.println(F("--- MT4 jog ---"));
  Serial.print(F("MODE="));
  Serial.print(cart_jog_mode ? F("cart") : F("joint"));
  Serial.print(F("  ORIENT="));
  Serial.print(cart_orient_hold ? F("hold") : F("free"));
  Serial.print(F("  HOMED="));
  Serial.print(mt4_homed ? F("yes") : F("no"));
  Serial.print(F("  SPEED="));
  Serial.println(dda_get_speed_us());
  Serial.print(F("GROUND_Z="));
  Serial.print(mt4_ground_z_mm, 1);
  Serial.print(F("  SOFT J1="));
  Serial.print(joint_soft_min[0]);
  Serial.print(F(".."));
  Serial.print(joint_soft_max[0]);
  Serial.print(F(" J2="));
  Serial.print(joint_soft_min[1]);
  Serial.print(F(".."));
  Serial.print(joint_soft_max[1]);
  Serial.print(F(" J3="));
  Serial.print(joint_soft_min[2]);
  Serial.print(F(".."));
  Serial.print(joint_soft_max[2]);
  Serial.print(F(" J4="));
  Serial.print(joint_soft_min[3]);
  Serial.print(F(".."));
  Serial.println(joint_soft_max[3]);
  Serial.print(F("J2+J3="));
  Serial.print(MT4_J2_J3_SUM_MIN);
  Serial.print(F(".."));
  Serial.print(MT4_J2_J3_SUM_MAX);
  Serial.print(F("  (extension couples J2/J3)"));
  Serial.println();
  print_joint_pos();
  Serial.print(F("STEP="));
  if (dda_axis_mask == 0) {
    Serial.println(F("none"));
  } else {
    bool first = true;
    for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
      if (!(dda_axis_mask & (1 << i))) {
        continue;
      }
      if (!first) {
        Serial.print('+');
      }
      Serial.print(F("D"));
      Serial.print(J_DRIVE[i]);
      first = false;
    }
    Serial.println();
  }
  Serial.print(F("EN="));
  Serial.print(drivers_enabled ? F("on") : F("off"));
  Serial.print(F("  JOG="));
  Serial.print(jog_active ? F("on") : F("off"));
  if (jog_active && dda_axis_mask != 0) {
    Serial.print(F(" T1"));
  }
  Serial.print(F("  LIM "));
  print_limits();
  Serial.print(F("  GRIP S="));
  Serial.print(gripper_s);
  Serial.print(F(" pwm="));
  Serial.print(gripper_pwm_on ? F("on") : F("off"));
  Serial.print(F(" sweep="));
  if (gripper_sweep == GRIP_SWEEP_OPEN) {
    Serial.println(F("open"));
  } else if (gripper_sweep == GRIP_SWEEP_CLOSE) {
    Serial.println(F("close"));
  } else {
    Serial.println(F("stop"));
  }
  Serial.println(F("---------------"));
}

void clear_jog_axes() {
  dda_clear_axes();
  cart_jog_mode = false;
}

bool add_jog_pin(uint8_t pin) {
  if (dda_axis_mask == 0) {
    cart_jog_mode = false;
  }
  return dda_add_axis_by_pin(pin);
}

bool remove_jog_pin(uint8_t pin) {
  return dda_remove_axis_by_pin(pin);
}

bool motion_start_jog() {
  if (dda_axis_mask == 0) {
    Serial.println(F("err no step pin"));
    return false;
  }
  dda_ramp_clear();  // a leftover `mp` ramp must not affect this jog's speed
  dda_engage();
  Serial.println(F("ok jog"));
  return true;
}

void motion_set_speed_us(long us) {
  dda_set_speed_us(us);
}

static void normalize_vec3(Vec3 *v) {
  const float n = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
  if (n < 1e-6f) {
    return;
  }
  v->x /= n;
  v->y /= n;
  v->z /= n;
}

static bool setup_cartesian_jog(const Vec3 *dir) {
  CartesianRates rates;
  const JointAnglesDeg q = angles_from_steps();

  /* Keep-out clamp: at the cylinder boundary, project away the inward
   * radial component so the jog slides along the cylinder (and up/down)
   * instead of driving the TCP into the base. Re-evaluated from the
   * CURRENT pose on every 40ms refresh, so the clamp engages and releases
   * automatically as the arm moves. Outward motion is always allowed
   * (that's the escape direction). */
  Vec3 d = *dir;
  {
    Vec3 tcp;
    mt4_fk_tcp(&q, &tcp);
    const float r = sqrtf(tcp.x * tcp.x + tcp.y * tcp.y);
    if (r > 1e-3f && r <= MT4_KEEPOUT_RADIUS_MM + 1.0f) {
      const float ux = tcp.x / r;
      const float uy = tcp.y / r;
      const float inward = d.x * ux + d.y * uy;
      if (inward < 0.0f) {
        d.x -= inward * ux;
        d.y -= inward * uy;
      }
    }
    /* Ground plane: block downward jog once at/below the desk envelope. */
    if (tcp.z <= mt4_ground_z_mm + 0.5f && d.z < 0.0f) {
      d.z = 0.0f;
    }
  }

  const bool has_dir = fabsf(d.x) + fabsf(d.y) + fabsf(d.z) > 1e-6f;
  if (has_dir) {
    if (!mt4_cartesian_rates(&q, &d, cart_orient_hold, &rates)) {
      return false;
    }
  } else {
    /* Pure J4 roll: nothing for the Cartesian solver to do (a zero
     * direction has no solution anyway). */
    rates.j1 = rates.j2 = rates.j3 = 0;
    rates.j4 = 0;
    rates.master = MT4_CJ_MASTER;
  }

  if (cart_j4_roll != 0) {
    /* Full-rate roll (one step per tick, same feel as the old standalone
     * J4 jog), added on top of any orient-hold counter-rotation and
     * clamped to the DDA's one-step-per-tick ceiling. Sign convention:
     * positive roll = positive joint direction (dda_arm derives the DIR
     * pin level from the delta's sign). */
    int32_t j4 = rates.j4 + static_cast<int32_t>(cart_j4_roll) * MT4_CJ_MASTER;
    if (j4 > MT4_CJ_MASTER) {
      j4 = MT4_CJ_MASTER;
    } else if (j4 < -MT4_CJ_MASTER) {
      j4 = -MT4_CJ_MASTER;
    }
    rates.j4 = j4;
  }

  /* Soft joint limits: any axis that would step past the envelope aborts
   * the whole Cartesian jog (do not keep sliding on the other axes). */
  {
    const int32_t rate_v[MT4_NUM_JOINTS] = {rates.j1, rates.j2, rates.j3,
                                            rates.j4};
    for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
      if (rate_v[i] == 0) {
        continue;
      }
      if (!motion_step_allowed(i, rate_v[i] > 0)) {
        return false;
      }
    }
  }

  if (rates.j1 == 0 && rates.j2 == 0 && rates.j3 == 0 && rates.j4 == 0) {
    return false;
  }
  cart_jog_mode = true;
  dda_ramp_clear();  // a leftover `mp` ramp must not affect this jog's speed
  const int32_t deltas[MT4_NUM_JOINTS] = {rates.j1, rates.j2, rates.j3,
                                          rates.j4};
  return dda_arm(rates.master, deltas, false);
}

void start_cartesian_jog(Vec3 dir, int8_t j4_roll) {
  mp_path_cancel();
  normalize_vec3(&dir);
  cart_dir_active = dir;
  cart_dir_active_valid = true;
  cart_j4_roll = j4_roll;
  cart_refresh_ms = millis();

  const bool was_active = jog_active;
  if (!was_active) {
    dda_stop();
    apply_all(PIN_FLOAT);
    set_enable(true);
    delay(ENABLE_SETTLE_MS);
  }

  if (!setup_cartesian_jog(&dir)) {
    // Soft-limit / keep-out / ground: refuse this direction without an
    // error line (client resends cj while the stick is held).
    dda_stop();
    cart_dir_active_valid = false;
    return;
  }

  if (!was_active) {
    dda_engage();
  } else {
    dda_refresh_pins();
  }
  Serial.println(F("ok cj"));
}

void refresh_cartesian_jog_if_due() {
  if (!cart_jog_mode || !cart_dir_active_valid || !jog_active) {
    return;
  }
  const unsigned long now = millis();
  if (now - cart_refresh_ms < CJ_REFRESH_MS) {
    return;
  }
  cart_refresh_ms = now;
  if (!setup_cartesian_jog(&cart_dir_active)) {
    // Soft-limit / keep-out / ground: stop the jog quietly. Returning false
    // here is expected at the envelope edge, not a solver failure.
    dda_stop();
    cart_dir_active_valid = false;
  } else {
    dda_refresh_pins();
  }
}

/* `m` command: bounded relative move of all joints (+ optional gripper
 * delta). Joint deltas are signed steps; the DDA runs each axis at a rate
 * proportional to its distance so all joints finish together. Replies
 * "ok m" immediately; "m done pos ..." is emitted from motion_poll_move_done
 * (called every loop()) when the joint motion completes. Drivers stay
 * enabled (holding) afterwards. */
void start_relative_move(const long d[MT4_NUM_JOINTS], long dg) {
  cart_jog_mode = false;
  move_done_is_mp = false;
  mp_path_cancel();
  dda_ramp_clear();  // a leftover `mp` ramp must not affect this move's speed

  int32_t master = 0;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    const int32_t mag = d[i] < 0 ? static_cast<int32_t>(-d[i])
                                 : static_cast<int32_t>(d[i]);
    if (mag > master) {
      master = mag;
    }
  }

  if (dg != 0) {
    gripperSweepToS(static_cast<long>(gripper_s) + dg);
  }

  if (master == 0) {
    /* gripper-only (or no-op) request: nothing to step */
    Serial.println(F("ok m"));
    report_move_done();
    return;
  }

  apply_all(PIN_FLOAT);
  set_enable(true);
  delay(ENABLE_SETTLE_MS);

  int32_t deltas[MT4_NUM_JOINTS];
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    deltas[i] = static_cast<int32_t>(d[i]);
  }
  dda_arm(master, deltas, true);
  dda_engage();
  Serial.println(F("ok m"));
}

/* "mp" command: absolute-position move along a straight world-frame
 * Cartesian line (TCP xyz). J4 is held fixed in world space when the
 * commanded value matches the pose at move start (J4 counters J1 1:1,
 * like `orient on`); otherwise J4 is interpolated linearly to the target.
 * The path is split into short segments; each segment solves IK and runs
 * a coordinated joint DDA move. Refused unless the arm has homed this session. */
bool start_absolute_move(float x, float y, float z, float j4_deg, long g,
                         long speed_us) {
  if (!mt4_homed) {
    Serial.println(F("err not homed"));
    return false;
  }
  if (g != 0 && !gripperSValid(g)) {
    Serial.print(F("err mp grip "));
    Serial.print(GRIPPER_S_OPEN);
    Serial.print('-');
    Serial.println(GRIPPER_S_CLOSED);
    return false;
  }
  if (speed_us != 0 &&
      (speed_us < JOG_STEP_PERIOD_MIN_US || speed_us > JOG_STEP_PERIOD_MAX_US)) {
    Serial.print(F("err mp speed "));
    Serial.print(JOG_STEP_PERIOD_MIN_US);
    Serial.print('-');
    Serial.println(JOG_STEP_PERIOD_MAX_US);
    return false;
  }

  motion_cancel_move();
  dda_stop();

  if (speed_us != 0) {
    dda_set_speed_us_quiet(speed_us);
  }

  if (sqrtf(x * x + y * y) < MT4_KEEPOUT_RADIUS_MM - 0.5f) {
    Serial.println(F("err mp keepout"));
    return false;
  }
  if (z < mt4_ground_z_mm - 0.05f) {
    Serial.print(F("err mp ground z<"));
    Serial.println(mt4_ground_z_mm, 1);
    return false;
  }

  const JointAnglesDeg near = angles_from_steps();
  const float ws_j4_now = mt4_ws_j4_deg(&near);
  const bool hold_ws_orient = fabsf(j4_deg - ws_j4_now) < 0.1f;

  JointAnglesDeg target;
  if (!mt4_ik_position(&near, x, y, z, 0.0f, &target)) {
    Serial.println(F("err mp unreachable"));
    return false;
  }
  target.j4 = j4_deg - target.j1;
  {
    long target_steps[MT4_NUM_JOINTS];
    angles_to_joint_steps(&target, target_steps);
    if (!motion_joints_within_soft_limits(target_steps)) {
      Serial.println(F("err mp joints"));
      return false;
    }
  }

  Vec3 start_tcp;
  mt4_fk_tcp(&near, &start_tcp);

  float xy_len = 0.0f;
  const uint8_t num_pieces =
      plan_keepout_path(start_tcp.x, start_tcp.y, x, y, mp_path.pieces, &xy_len);

  const float dz = z - start_tcp.z;
  const float cart_dist = sqrtf(xy_len * xy_len + dz * dz);

  uint16_t num_segments =
      static_cast<uint16_t>(ceilf(cart_dist / MP_CART_SEGMENT_MM));
  if (num_segments < 1) {
    num_segments = 1;
  }
  if (num_segments > MP_MAX_SEGMENTS) {
    num_segments = MP_MAX_SEGMENTS;
  }

  mp_path.sz = start_tcp.z;
  mp_path.sj4_ws = ws_j4_now;
  mp_path.ez = z;
  mp_path.ej4_ws = j4_deg;
  mp_path.hold_ws_orient = hold_ws_orient;
  mp_path.num_pieces = num_pieces;
  mp_path.total_xy_len = xy_len;
  mp_path.num_segments = num_segments;

  const int32_t master_total = mp_estimate_path_ticks();
  dda_set_ramp(MP_ACCEL_START_US, dda_get_speed_us(), master_total,
               MP_ACCEL_RAMP_TICKS);

  cart_jog_mode = false;
  move_done_is_mp = true;

  mp_path.next_seg = 1;
  mp_path.active = true;

  if (g != 0) {
    gripperSweepToS(g);
  }

  Serial.println(F("ok mp"));
  return mp_continue_path();
}

void motion_poll_move_done() {
  if (!move_done_pending) {
    return;
  }
  move_done_pending = false;
  dda_stop();
  move_mode = false;

  if (mp_path.active) {
    mp_continue_path();
    return;
  }
  report_move_done();
}
