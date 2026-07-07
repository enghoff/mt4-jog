#include "motion.h"

#include <math.h>

#include "dda.h"
#include "gripper.h"
#include "pins.h"

bool cart_orient_hold = true;
bool mt4_homed = false;

static bool cart_jog_mode = false;
static Vec3 cart_dir_active = {0.0f, 0.0f, 0.0f};
static bool cart_dir_active_valid = false;
static unsigned long cart_refresh_ms = 0;

static JointAnglesDeg angles_from_steps() {
  JointAnglesDeg q;
  q.j1 = MT4_HOME_J1_DEG + MT4_J1_STEP_SIGN *
                               static_cast<float>(joint_steps[0]) /
                               MT4_STEPS_PER_DEG[0];
  q.j2 = MT4_HOME_J2_DEG + MT4_J2_STEP_SIGN *
                               static_cast<float>(joint_steps[1]) /
                               MT4_STEPS_PER_DEG[1];
  q.j3 = MT4_HOME_J3_DEG + MT4_J3_STEP_SIGN *
                               static_cast<float>(joint_steps[2]) /
                               MT4_STEPS_PER_DEG[2];
  q.j4 = MT4_HOME_J4_DEG + MT4_J4_STEP_SIGN *
                               static_cast<float>(joint_steps[3]) /
                               MT4_STEPS_PER_DEG[3];
  return q;
}

void motion_init() {
  dda_init();
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
  // the base under J1's pivot; j4 deg absolute; grip S).
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
  Serial.print(q.j4, 1);
  Serial.print(F(" grip="));
  Serial.println(gripper_s);
}

// F()'s PSTR() expands to a GCC statement-expression, which can't be used
// as a static initializer at file scope -- so track which command is
// pending with a plain bool instead of caching the F() string itself.
static bool move_done_is_mp = false;

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
  Serial.println(mt4_homed ? F("yes") : F("no"));
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
  const JointAnglesDeg q = angles_from_steps();
  CartesianRates rates;
  if (!mt4_cartesian_rates(&q, dir, cart_orient_hold, &rates)) {
    return false;
  }
  cart_jog_mode = true;
  const int32_t deltas[MT4_NUM_JOINTS] = {rates.j1, rates.j2, rates.j3,
                                          rates.j4};
  return dda_arm(rates.master, deltas, false);
}

void start_cartesian_jog(Vec3 dir) {
  normalize_vec3(&dir);
  cart_dir_active = dir;
  cart_dir_active_valid = true;
  cart_refresh_ms = millis();

  const bool was_active = jog_active;
  if (!was_active) {
    dda_stop();
    apply_all(PIN_FLOAT);
    set_enable(true);
    delay(ENABLE_SETTLE_MS);
  }

  if (!setup_cartesian_jog(&dir)) {
    Serial.println(F("err cj"));
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
    dda_stop();
    Serial.println(F("err cj refresh"));
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

/* "mp" command: absolute-position move. Uses the same coordinated-DDA
 * machinery as start_relative_move(), but the joint targets come from
 * mt4_ik_position() instead of a caller-supplied step delta, and the move
 * is refused outright unless the arm has homed this session -- absolute
 * coordinates are meaningless relative to an unreferenced step counter. */
bool start_absolute_move(float x, float y, float z, float j4_deg, long g) {
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

  const JointAnglesDeg near = angles_from_steps();
  JointAnglesDeg target;
  if (!mt4_ik_position(&near, x, y, z, j4_deg, &target)) {
    Serial.println(F("err mp unreachable"));
    return false;
  }

  const long target_steps[MT4_NUM_JOINTS] = {
      lroundf((target.j1 - MT4_HOME_J1_DEG) * MT4_STEPS_PER_DEG[0] *
              MT4_J1_STEP_SIGN),
      lroundf((target.j2 - MT4_HOME_J2_DEG) * MT4_STEPS_PER_DEG[1] *
              MT4_J2_STEP_SIGN),
      lroundf((target.j3 - MT4_HOME_J3_DEG) * MT4_STEPS_PER_DEG[2] *
              MT4_J3_STEP_SIGN),
      lroundf((target.j4 - MT4_HOME_J4_DEG) * MT4_STEPS_PER_DEG[3] *
              MT4_J4_STEP_SIGN),
  };

  int32_t deltas[MT4_NUM_JOINTS];
  int32_t master = 0;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    const long d = target_steps[i] - joint_steps[i];
    if (d > MOVE_MAX_STEPS || d < -MOVE_MAX_STEPS) {
      Serial.println(F("err mp step delta too large"));
      return false;
    }
    deltas[i] = static_cast<int32_t>(d);
    const int32_t mag = deltas[i] < 0 ? -deltas[i] : deltas[i];
    if (mag > master) {
      master = mag;
    }
  }

  cart_jog_mode = false;
  move_done_is_mp = true;

  if (g != 0) {
    gripperSweepToS(g);
  }

  if (master == 0) {
    Serial.println(F("ok mp"));
    report_move_done();
    return true;
  }

  apply_all(PIN_FLOAT);
  set_enable(true);
  delay(ENABLE_SETTLE_MS);
  dda_arm(master, deltas, true);
  dda_engage();
  Serial.println(F("ok mp"));
  return true;
}

void motion_poll_move_done() {
  if (!move_done_pending) {
    return;
  }
  move_done_pending = false;
  dda_stop();
  move_mode = false;
  report_move_done();
}
