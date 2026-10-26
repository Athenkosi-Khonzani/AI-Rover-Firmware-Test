# =========================================================================
# motion.py - Micromouse motion primitives
#
# forward(distance_m)  - closed-loop straight line        (encoders + PID)
# turn(angle_deg)      - spin in place, +left / -right    (gyro + PID)
#
# forward() uses TWO PID loops, both driven by the wheel encoders only:
#
#   1. DISTANCE loop : average of both wheels  -> base speed (PWM)
#        error = target - average distance travelled
#        Uses derivative-on-measurement (speed) so it brakes smoothly.
#
#   2. STRAIGHTNESS loop : left distance - right distance -> steering (PWM)
#        error = left_dist - right_dist   (0 when driving straight)
#        left_pwm  = base - steer
#        right_pwm = base + steer
#        The integral term removes the constant drift caused by
#        motor imbalance.
#
# Run this file directly to test:   TEST = "forward"  (bottom of file)
# =========================================================================

import sys
import uct_mouse

# On the real mouse delay_ms() paces real time, so we measure dt with the
# clock.  In the PC simulator, physics advances 10 ms per step regardless of
# wall-clock time, so we use the fixed loop period instead.
ON_MOUSE = sys.implementation.name == "micropython"

try:                                    # MicroPython (on the mouse)
    from time import ticks_ms, ticks_diff
except ImportError:                     # normal Python (PC simulator)
    import time

    def ticks_ms():
        return int(time.time() * 1000)

    def ticks_diff(a, b):
        return a - b


# -------------------------------------------------------------------------
# CONFIGURATION  (all starting values - tune them!)
# -------------------------------------------------------------------------

# Encoder scale.  PLACEHOLDER from the simulator config (1170 ticks per
# wheel revolution, 32.5 mm wheel radius).  Calibrate on the real mouse
# with calibrate_ticks() and put your number here.
TICKS_PER_M = 1170.0 / (2.0 * 3.14159265 * 0.0325)      # ~5730 ticks / m

# Motor deadband: the smallest PWM (%) that actually turns the wheels.
# The simulator uses ~60.  Measure yours with find_deadband().
# forward() works in "effort" 0..100 and maps it onto DEADBAND..100 PWM,
# so effort 0 = stopped and effort 1 = just moving.
DEADBAND = 62

LOOP_MS = 10                # control loop period

# Distance PID (units: effort per metre of error)
DIST_KP = 1200.0            # full cruise until ~3 cm from the target
DIST_KI = 0.0               # keep 0: it winds up against the deadband and overshoots
DIST_KD = 60.0              # effort per (m/s): brakes as speed builds

# Straightness PID (units: effort per metre of left-right difference)
STEER_KP = 400.0
STEER_KI = 150.0            # removes the steady drift from motor imbalance
STEER_KD = 10.0
STEER_I_MAX = 15.0          # anti-windup clamp on the integral (effort)

# Turning PID (units: effort per degree of heading error)
TURN_KP = 1.5
TURN_KI = 4.0                # overcomes stiction near the target
TURN_KD = 0.12              # effort per (deg/s) - uses the gyro rate directly
TURN_SPEED_DEFAULT = 40     # max turning effort 0..100
TURN_TOL_DEG = 1.0          # "arrived" if within 1 degree ...
TURN_SETTLE_DPS = 5.0       # ... and rotating slower than 5 deg/s ...

GYRO_BIAS = None            # deg/s, measured by calibrate_gyro()
GYRO_SCALE = 0.5            # gyro over-reads 2x on this mouse (measured), so halve it

TRACK_M = 0.109             # wheel-to-wheel distance (m) for encoder turning
TURN_BAL_KP = 400.0         # effort per metre of net drift while spinning (0 = off)

SPEED_DEFAULT = 40          # cruise effort 0..100  (100 ~ 0.4 m/s in the sim)
ACCEL_PER_S = 200.0         # slew limit: how fast the effort may ramp up
TOL_M_DEFAULT = 0.005       # "arrived" if within 5 mm ...
SETTLE_SPEED = 0.02         # ... and slower than 2 cm/s ...
SETTLE_LOOPS = 5            # ... for this many loops in a row


# -------------------------------------------------------------------------
# A small PID controller
# -------------------------------------------------------------------------
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, out_min=-100.0, out_max=100.0,
                 i_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_max = i_max if i_max is not None else out_max
        self.reset()

    def reset(self):
        self.integral = 0.0          # already multiplied by ki (PWM units)
        self.prev_error = None

    def update(self, error, dt, rate=None):
        """Return the controller output.

        rate: optional measured rate of change of the *measurement*
              (e.g. speed).  If given, the D term is  -kd * rate
              (derivative on measurement, no kick when the target changes).
              If omitted, D is taken from the change in error.
        """
        p = self.kp * error

        self.integral += self.ki * error * dt
        if self.integral > self.i_max:
            self.integral = self.i_max
        elif self.integral < -self.i_max:
            self.integral = -self.i_max

        if rate is not None:
            d = -self.kd * rate
        elif self.prev_error is not None and dt > 0:
            d = self.kd * (error - self.prev_error) / dt
        else:
            d = 0.0
        self.prev_error = error

        out = p + self.integral + d
        if out > self.out_max:
            out = self.out_max
        elif out < self.out_min:
            out = self.out_min
        return out


# -------------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------------
def setup():
    """Initialise the mouse and load polarity.txt if it exists."""
    if not uct_mouse.init():
        print("Initialization failed.")
        return False
    # Only override the board's own polarity if YOU created polarity.txt
    # (format: motorL,motorR,encL,encR   e.g.  -1,-1,1,1).  Otherwise leave
    # the firmware defaults alone - they are already set for this chassis.
    try:
        with open("polarity.txt", "r") as f:
            v = f.read().strip().split(",")
        uct_mouse.set_polarity(int(v[0]), int(v[1]))
        if len(v) >= 4:
            uct_mouse.set_encoder_polarity(int(v[2]), int(v[3]))
        print("polarity.txt loaded:", v)
    except Exception:
        print("no polarity.txt - using the board's default polarity")
    return True


def stop():
    uct_mouse.set_motors(0, 0)


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _effort_to_pwm(effort, deadband):
    """Linearise the motor: effort 0 -> 0 PWM, effort 1..100 -> deadband..100."""
    if effort > 0.5:
        return int(_clamp(deadband + effort * (100 - deadband) / 100.0, 0, 100))
    if effort < -0.5:
        return int(_clamp(-deadband + effort * (100 - deadband) / 100.0, -100, 0))
    return 0


# -------------------------------------------------------------------------
# forward()
# -------------------------------------------------------------------------
def forward(distance_m, speed=SPEED_DEFAULT, tol_m=TOL_M_DEFAULT,
            timeout_s=None, log=False, deadband=None, abort_check=None):
    """Drive in a straight line for distance_m metres (negative = reverse).

    speed: cruise effort 0..100 (not raw PWM - see DEADBAND).

    Returns (distance_travelled_m, elapsed_s).
    Set log=True to print a CSV of the run afterwards (for tuning/plotting).
    abort_check: optional function called every loop; return True to stop
    the motion early (used by remote.py for a "stop" command).
    """
    if timeout_s is None:
        timeout_s = 3.0 + 10.0 * abs(distance_m)

    if deadband is None:
        deadband = DEADBAND
    dt_nominal = LOOP_MS / 1000.0
    dist_pid = PID(DIST_KP, DIST_KI, DIST_KD, -speed, speed)
    steer_pid = PID(STEER_KP, STEER_KI, STEER_KD, -30, 30, STEER_I_MAX)

    l0, r0 = uct_mouse.get_encoders()
    t_start = ticks_ms()
    t_prev = t_start
    elapsed = 0.0
    dist_prev = 0.0
    base_cmd = 0.0
    settled = 0
    stall_t = 0.0
    rows = []
    n = 0
    result = (0.0, 0.0)
    aborted = None

    try:
        while True:
            uct_mouse.delay_ms(LOOP_MS)         # paces sensor updates too

            # ---- time step ------------------------------------------------
            if ON_MOUSE:
                now = ticks_ms()
                dt = ticks_diff(now, t_prev) / 1000.0
                if dt <= 0:
                    dt = dt_nominal
                t_prev = now
            else:
                dt = dt_nominal
            elapsed += dt

            # ---- measure --------------------------------------------------
            l, r = uct_mouse.get_encoders()
            dl = (l - l0) / TICKS_PER_M         # left distance  (m)
            dr = (r - r0) / TICKS_PER_M         # right distance (m)
            dist = (dl + dr) / 2.0
            speed = (dist - dist_prev) / dt     # m/s
            dist_prev = dist
            err = distance_m - dist

            # ---- safety: stop command / timeout / wrong encoder direction --
            if abort_check is not None and abort_check():
                aborted = "stopped by command"
                break
            if elapsed > timeout_s:
                aborted = "timeout"
                break
            if elapsed > 0.6 and abs(dist) < 1.0 / TICKS_PER_M:
                aborted = "no encoder movement (motors off? wrong polarity?)"
                break
            if elapsed > 0.6 and dist * distance_m < 0 and abs(dist) > 0.01:
                aborted = "moving the wrong way - check set_polarity / " \
                          "set_encoder_polarity"
                break

            # stalled: pushing but not moving (blocked by a wall?)
            if abs(base_cmd) > 15 and abs(speed) < 0.01:
                stall_t += dt
                if stall_t > 1.0:
                    aborted = "stalled - pushing but not moving (blocked?)"
                    break
            else:
                stall_t = 0.0

            # ---- arrived? -------------------------------------------------
            if abs(err) < tol_m:
                if abs(speed) < SETTLE_SPEED:
                    settled += 1
                    if settled >= SETTLE_LOOPS:
                        break
                else:
                    settled = 0
                uct_mouse.set_motors(0, 0)      # in the window: brake/coast
                steer_pid.reset()
                base_cmd = 0.0
                n += 1
                continue
            settled = 0

            # ---- loop 1: distance -> base speed ---------------------------
            base_target = dist_pid.update(err, dt, rate=speed)

            # slew-rate limit on speeding up (avoids wheel slip at the start)
            step = ACCEL_PER_S * dt
            if base_target > base_cmd + step:
                base_cmd += step
            elif base_target < base_cmd - step:
                base_cmd -= step
            else:
                base_cmd = base_target

            # ---- loop 2: straightness -> steering -------------------------
            steer = steer_pid.update(dl - dr, dt)

            left_pwm = _effort_to_pwm(base_cmd - steer, deadband)
            right_pwm = _effort_to_pwm(base_cmd + steer, deadband)
            uct_mouse.set_motors(left_pwm, right_pwm)

            # ---- log (compact ints, every 2nd loop, capped) ---------------
            if log and n % 2 == 0 and len(rows) < 600:
                rows.append((int(elapsed * 1000), int(dist * 1000),
                             int(err * 1000), int(speed * 1000),
                             int(dl * 1000), int(dr * 1000),
                             int(base_cmd), int(steer),
                             left_pwm, right_pwm))
            n += 1
    finally:
        uct_mouse.set_motors(0, 0)              # ALWAYS stop, even on Ctrl+C

    # let it finish rolling, then read the final position
    uct_mouse.delay_ms(200)
    l, r = uct_mouse.get_encoders()
    final = ((l - l0) + (r - r0)) / 2.0 / TICKS_PER_M
    total_s = elapsed
    result = (final, total_s)

    if aborted:
        print("ABORTED:", aborted)
    print("forward: target %.3f m, got %.3f m (error %.1f mm) in %.2f s"
          % (distance_m, final, (distance_m - final) * 1000.0, total_s))
    print("         left %.3f m, right %.3f m (difference %.1f mm)"
          % ((l - l0) / TICKS_PER_M, (r - r0) / TICKS_PER_M,
             ((l - l0) - (r - r0)) / TICKS_PER_M * 1000.0))

    if log:
        print("t_ms,dist_mm,err_mm,speed_mms,dl_mm,dr_mm,base_effort,steer,lpwm,rpwm")
        for row in rows:
            print(",".join([str(v) for v in row]))
    return result


# -------------------------------------------------------------------------
# Gyro helpers
# -------------------------------------------------------------------------
def calibrate_gyro(samples=60):
    """Measure the gyro's zero-rate offset.  KEEP THE MOUSE STILL (~0.6 s)."""
    global GYRO_BIAS
    total = 0.0
    for _ in range(samples):
        uct_mouse.delay_ms(LOOP_MS)
        total += uct_mouse.get_gyro()
    GYRO_BIAS = total / samples
    print("gyro bias: %.3f deg/s" % GYRO_BIAS)
    return GYRO_BIAS


def test_gyro(seconds=6):
    """Hold the mouse IN THE AIR and rotate it by hand (wheels not touching).

    Prints the integrated heading.  Turn it 90 degrees anticlockwise (seen
    from above): it should read about +90.  Clockwise should read negative.
    If the size is wrong, the gyro units/scale differ; if the sign is wrong,
    negate the gyro in turn().
    """
    calibrate_gyro()
    print("rotate it now ...")
    heading = 0.0
    t = 0.0
    last_print = 0.0
    while t < seconds:
        uct_mouse.delay_ms(LOOP_MS)
        t += LOOP_MS / 1000.0
        heading += (uct_mouse.get_gyro() - GYRO_BIAS) * GYRO_SCALE * LOOP_MS / 1000.0
        if t - last_print >= 0.5:
            last_print = t
            print("heading: %7.1f deg" % heading)


# -------------------------------------------------------------------------
# turn()
# -------------------------------------------------------------------------
def turn(angle_deg, speed=TURN_SPEED_DEFAULT, tol_deg=TURN_TOL_DEG,
         timeout_s=None, log=False, deadband=None, abort_check=None):
    """Spin on the spot by angle_deg.  Positive = LEFT (anticlockwise),
    negative = RIGHT.  Uses the gyro (integrated yaw) as feedback.

    Returns (angle_turned_deg, elapsed_s).
    """
    if GYRO_BIAS is None:
        print("(measuring gyro bias - keep the mouse still)")
        calibrate_gyro()
    if deadband is None:
        deadband = DEADBAND
    if timeout_s is None:
        timeout_s = 3.0 + abs(angle_deg) / 30.0

    dt_nominal = LOOP_MS / 1000.0
    pid = PID(TURN_KP, TURN_KI, TURN_KD, -speed, speed, 10.0)

    l0, r0 = uct_mouse.get_encoders()
    t_prev = ticks_ms()
    elapsed = 0.0
    heading = 0.0
    settled = 0
    stall_t = 0.0
    cmd = 0.0
    rows = []
    n = 0
    aborted = None

    try:
        while True:
            uct_mouse.delay_ms(LOOP_MS)

            if ON_MOUSE:
                now = ticks_ms()
                dt = ticks_diff(now, t_prev) / 1000.0
                if dt <= 0:
                    dt = dt_nominal
                t_prev = now
            else:
                dt = dt_nominal
            elapsed += dt

            rate = (uct_mouse.get_gyro() - GYRO_BIAS) * GYRO_SCALE   # deg/s, + = left
            heading += rate * dt
            err = angle_deg - heading

            if abort_check is not None and abort_check():
                aborted = "stopped by command"
                break
            if elapsed > timeout_s:
                aborted = "timeout"
                break
            if elapsed > 0.6 and heading * angle_deg < 0 and abs(heading) > 5:
                aborted = ("turning the wrong way - check the gyro sign "
                           "(test_gyro) and motor polarity")
                break
            if abs(cmd) > 15 and abs(rate) < 3.0:
                stall_t += dt
                if stall_t > 1.0:
                    aborted = "not rotating - motors too weak? (raise DEADBAND)"
                    break
            else:
                stall_t = 0.0

            if abs(err) < tol_deg:
                if abs(rate) < TURN_SETTLE_DPS:
                    settled += 1
                    if settled >= SETTLE_LOOPS:
                        break
                else:
                    settled = 0
                uct_mouse.set_motors(0, 0)
                pid.reset()
                cmd = 0.0
                n += 1
                continue
            settled = 0

            target = pid.update(err, dt, rate=rate)
            step = ACCEL_PER_S * dt
            if target > cmd + step:
                cmd += step
            elif target < cmd - step:
                cmd -= step
            else:
                cmd = target

            # left turn: left wheel backward, right wheel forward
            uct_mouse.set_motors(_effort_to_pwm(-cmd, deadband),
                                 _effort_to_pwm(cmd, deadband))

            if log and n % 2 == 0 and len(rows) < 600:
                rows.append((int(elapsed * 1000), int(heading * 10),
                             int(err * 10), int(rate), int(cmd)))
            n += 1
    finally:
        uct_mouse.set_motors(0, 0)

    # let it stop rotating, and keep integrating while it does
    for _ in range(20):
        uct_mouse.delay_ms(LOOP_MS)
        heading += (uct_mouse.get_gyro() - GYRO_BIAS) * GYRO_SCALE * dt_nominal
    l, r = uct_mouse.get_encoders()
    dl = (l - l0) / TICKS_PER_M
    dr = (r - r0) / TICKS_PER_M

    if aborted:
        print("ABORTED:", aborted)
    print("turn: target %.1f deg, got %.1f deg (error %.1f) in %.2f s"
          % (angle_deg, heading, angle_deg - heading, elapsed))
    enc_angle = (dr - dl) / TRACK_M * 180.0 / 3.14159265
    print("      encoders say the mouse turned %.1f deg (track %.0f mm)"
          % (enc_angle, TRACK_M * 1000))
    if abs(heading) > 5:
        print("      gyro/encoder ratio %.2f  (about 2.0 => gyro reads double:"
              " set GYRO_SCALE = 0.5)" % (heading / enc_angle
                                          if abs(enc_angle) > 1 else 0))
    if log:
        print("t_ms,heading_x10,err_x10,rate_dps,cmd_effort")
        for row in rows:
            print(",".join([str(v) for v in row]))
    return (heading, elapsed)


def turn_enc(angle_deg, speed=TURN_SPEED_DEFAULT, tol_deg=TURN_TOL_DEG,
             timeout_s=None, log=False, deadband=None, abort_check=None):
    """Spin on the spot using the ENCODERS ONLY (no gyro).  +left / -right.

    Each wheel's travel gives the heading of a differential-drive robot
    turning in place:   heading = (right_dist - left_dist) / TRACK_M   (rad)

    Returns (angle_turned_deg, elapsed_s).
    """
    if deadband is None:
        deadband = DEADBAND
    if timeout_s is None:
        timeout_s = 3.0 + abs(angle_deg) / 30.0

    k = 180.0 / 3.14159265 / TRACK_M / TICKS_PER_M      # ticks -> degrees
    dt_nominal = LOOP_MS / 1000.0
    pid = PID(TURN_KP, TURN_KI, TURN_KD, -speed, speed, 10.0)

    l0, r0 = uct_mouse.get_encoders()
    t_prev = ticks_ms()
    elapsed = 0.0
    heading = 0.0
    heading_prev = 0.0
    rate = 0.0                          # low-pass filtered deg/s
    settled = 0
    stall_t = 0.0
    cmd = 0.0
    rows = []
    n = 0
    aborted = None

    try:
        while True:
            uct_mouse.delay_ms(LOOP_MS)

            if ON_MOUSE:
                now = ticks_ms()
                dt = ticks_diff(now, t_prev) / 1000.0
                if dt <= 0:
                    dt = dt_nominal
                t_prev = now
            else:
                dt = dt_nominal
            elapsed += dt

            l, r = uct_mouse.get_encoders()
            heading = ((r - r0) - (l - l0)) * k          # degrees, + = left
            raw_rate = (heading - heading_prev) / dt
            heading_prev = heading
            rate += 0.4 * (raw_rate - rate)              # smooth quantisation
            err = angle_deg - heading

            if abort_check is not None and abort_check():
                aborted = "stopped by command"
                break
            if elapsed > timeout_s:
                aborted = "timeout"
                break
            if elapsed > 0.6 and heading * angle_deg < 0 and abs(heading) > 5:
                aborted = ("turning the wrong way - check motor and "
                           "encoder polarity")
                break
            if abs(cmd) > 15 and abs(rate) < 3.0:
                stall_t += dt
                if stall_t > 1.0:
                    aborted = "not rotating - motors too weak? (raise DEADBAND)"
                    break
            else:
                stall_t = 0.0

            if abs(err) < tol_deg:
                if abs(rate) < TURN_SETTLE_DPS:
                    settled += 1
                    if settled >= SETTLE_LOOPS:
                        break
                else:
                    settled = 0
                uct_mouse.set_motors(0, 0)
                pid.reset()
                cmd = 0.0
                n += 1
                continue
            settled = 0

            target = pid.update(err, dt, rate=rate)
            step = ACCEL_PER_S * dt
            if target > cmd + step:
                cmd += step
            elif target < cmd - step:
                cmd -= step
            else:
                cmd = target

            # spin in place: net travel (left + right) should stay at zero,
            # so push both wheels back if the mouse is creeping forward
            drift = ((r - r0) + (l - l0)) / TICKS_PER_M
            bal = _clamp(TURN_BAL_KP * drift, -10, 10)
            uct_mouse.set_motors(_effort_to_pwm(-cmd - bal, deadband),
                                 _effort_to_pwm(cmd - bal, deadband))

            if log and n % 2 == 0 and len(rows) < 600:
                rows.append((int(elapsed * 1000), int(heading * 10),
                             int(err * 10), int(rate), int(cmd)))
            n += 1
    finally:
        uct_mouse.set_motors(0, 0)

    uct_mouse.delay_ms(200)                     # let it roll to a stop
    l, r = uct_mouse.get_encoders()
    heading = ((r - r0) - (l - l0)) * k

    if aborted:
        print("ABORTED:", aborted)
    print("turn_enc: target %.1f deg, got %.1f deg (error %.1f) in %.2f s"
          % (angle_deg, heading, angle_deg - heading, elapsed))
    print("          wheel travel: left %.1f mm, right %.1f mm  (track %.0f mm)"
          % ((l - l0) / TICKS_PER_M * 1000, (r - r0) / TICKS_PER_M * 1000,
             TRACK_M * 1000))
    if log:
        print("t_ms,heading_x10,err_x10,rate_dps,cmd_effort")
        for row in rows:
            print(",".join([str(v) for v in row]))
    return (heading, elapsed)


def left(angle_deg, enc=False, **kw):
    f = turn_enc if enc else turn
    return f(abs(angle_deg), **kw)


def right(angle_deg, enc=False, **kw):
    f = turn_enc if enc else turn
    return f(-abs(angle_deg), **kw)


# -------------------------------------------------------------------------
# Calibration helper: find TICKS_PER_M for YOUR mouse
# -------------------------------------------------------------------------
def calibrate_ticks(pwm=30, seconds=1.0):
    """Open-loop drive, then print encoder ticks.

    1. Put the mouse on the floor with ~1 m clear in front.
    2. Mark where it starts, run this, then measure how far it rolled.
    3. TICKS_PER_M = ticks / distance_in_metres   (put it at the top).

    Also a good polarity check: with the mouse lifted, BOTH tick counts
    should be POSITIVE when the wheels spin forward.
    """
    l0, r0 = uct_mouse.get_encoders()
    steps = int(seconds * 10)
    for _ in range(steps):                      # re-send: 1 s watchdog
        uct_mouse.set_motors(pwm, pwm)
        uct_mouse.delay_ms(100)
    uct_mouse.set_motors(0, 0)
    uct_mouse.delay_ms(500)
    l1, r1 = uct_mouse.get_encoders()
    print("left ticks: %d   right ticks: %d" % (l1 - l0, r1 - r0))
    print("Measure the distance rolled (m):  TICKS_PER_M = ticks / distance")


def find_deadband(start=30, stop=100, hold_ms=250):
    """Ramp the PWM up slowly until the wheels start turning.

    Put the mouse on the floor (or lift it) with room to move.  Prints the
    PWM at which each wheel first moved - put a value slightly above the
    larger one into DEADBAND.
    """
    l0, r0 = uct_mouse.get_encoders()
    found_l = found_r = None
    pwm = start
    try:
        while pwm <= stop and (found_l is None or found_r is None):
            uct_mouse.set_motors(pwm, pwm)
            uct_mouse.delay_ms(hold_ms)
            l, r = uct_mouse.get_encoders()
            if found_l is None and abs(l - l0) > 20:
                found_l = pwm
            if found_r is None and abs(r - r0) > 20:
                found_r = pwm
            pwm += 2
    finally:
        uct_mouse.set_motors(0, 0)
    print("wheels first moved at PWM: left %s, right %s" % (found_l, found_r))
    print("Set DEADBAND a couple of points above the larger of the two.")


# -------------------------------------------------------------------------
# Run this file directly to test
# -------------------------------------------------------------------------
TEST = "forward"   # deadband | calibrate | forward | gyro | turn | turn_enc | square | square_enc | spin360

if __name__ == "__main__":
    if setup():
        if TEST == "deadband":
            find_deadband()
        elif TEST == "calibrate":
            calibrate_ticks(pwm=DEADBAND + 10)
        elif TEST == "gyro":
            test_gyro()
        elif TEST == "turn":
            print("Turning left 90 ...")
            turn(90, log=True)
        elif TEST == "turn_enc":
            print("Turning left 90 with the encoders ...")
            turn_enc(90, log=True)
        elif TEST == "spin360":
            print("Spinning left 360 with the encoders - measure the real angle")
            turn_enc(360)
        elif TEST == "square_enc":
            for _ in range(4):
                forward(0.5)
                uct_mouse.delay_ms(200)
                turn_enc(90)
                uct_mouse.delay_ms(200)
        elif TEST == "square":
            calibrate_gyro()
            for _ in range(4):
                forward(0.5)
                uct_mouse.delay_ms(200)
                turn(90)
                uct_mouse.delay_ms(200)
        else:
            print("Driving forward 0.5 m ...")
            forward(0.5, log=True)
