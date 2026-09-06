"""Switched-state inverting buck-boost converter with digital PID control.

Model (ideal diode, zero-current boundary for DCM):
  ON  (S closed,  D off):  diL/dt = +Vin/L ;   dvC/dt = -vC/(RC)
  OFF (S open,    D on):   diL/dt =  vC/L ;    dvC/dt = -(iL + vC/R)/C

Checks that hold exactly in CCM steady state:
  vC = -D/(1-D)*Vin ;  iL = -vC/(R(1-D)) ;  P_in = P_out
"""
import numpy as np

D_MIN, D_MAX = 0.05, 0.95


class DigitalPID:
    """Velocity-form (incremental) PID. e[k] = Vmeas[k] - Vref, positive gains.

    du = Kp*(e-e1) + Ki*Ts*e + (Kd/Ts)*(e - 2e1 + e2)
    d  = clamp(d + du, D_MIN, D_MAX)
    Saturation is applied to the duty command only; there is no separate
    accumulating integral state (integrator lives implicitly in d).
    """

    def __init__(self, kp, ki, kd, ts):
        self.kp, self.ki, self.kd, self.ts = kp, ki, kd, ts
        self.e1 = self.e2 = 0.0
        self.d = D_MIN

    def reset(self, d0=0.5):
        self.e1 = self.e2 = 0.0
        self.d = d0

    def step(self, vmeas, vref):
        e = vmeas - vref
        du = self.kp * (e - self.e1) \
             + self.ki * self.ts * e \
             + (self.kd / self.ts) * (e - 2 * self.e1 + self.e2)
        self.e2, self.e1 = self.e1, e
        self.d = min(D_MAX, max(D_MIN, self.d + du))
        return self.d


def quantize_abs(x, bits, fs):
    """Uniform mid-tread quantizer: y = round(x/fs*2^bits)*fs/2^bits."""
    q = 2.0 ** bits
    return np.round(x / fs * q) / q * fs


class ContinuousPID:
    """True analog PID (for the continuous-ideal baseline reference):
    u = Kp*e + Ki*int(e) + Kd*de/dt, duty clamped to [D_MIN, D_MAX]."""

    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.e_prev = 0.0
        self.d = D_MIN

    def reset(self, d0=0.5):
        self.integral = 0.0
        self.e_prev = 0.0
        self.d = d0

    def step(self, vmeas, vref, dt):
        e = vmeas - vref
        self.integral += self.ki * e * dt
        deriv = (e - self.e_prev) / dt
        self.e_prev = e
        self.d = min(D_MAX, max(D_MIN, self.kp * e + self.integral
                                + self.kd * deriv))
        return self.d


class BuckBoost:
    """Fixed-step Euler switched-model simulation, discrete controller.

    Optional audit knobs (default 0 = original zero-delay, noiseless model):
      comp_delay_s: computational delay before a computed duty takes effect
      adc_noise_sigma: Gaussian sigma (V, output-frame) added to vC before ADC
    """

    def __init__(self, vin=12.0, l=100e-6, c=220e-6, fsw=100e3, r=10.0,
                 ts_mult=1, adc_bits=16, pwm_bits=16, adc_fs=24.0,
                 pid=None, dt_div=200, comp_delay_s=0.0, adc_noise_sigma=0.0):
        self.vin, self.l, self.c, self.fsw, self.r = vin, l, c, fsw, r
        self.ts = ts_mult / fsw
        self.adc_bits, self.pwm_bits, self.adc_fs = adc_bits, pwm_bits, adc_fs
        self.adc_ideal = adc_bits >= 16
        self.pwm_ideal = pwm_bits >= 16
        self.pid = pid or DigitalPID(0.1, 2000, 5e-5, self.ts)
        self.dt = 1.0 / (fsw * dt_div)
        self.steps_per_cycle = max(1, round(1.0 / (fsw * self.dt)))
        self.comp_delay_s = comp_delay_s
        self.adc_noise_sigma = adc_noise_sigma
        self._delay_steps = int(round(comp_delay_s / self.dt)) if comp_delay_s > 0 else 0

    def run(self, t_end, vref, vmeas_scale=1.0, dist=None, log_div=40, t0=0.0):
        """dist: callable(t, iL, vC) -> (new_vin, new_r) applied at each step."""
        pid = self.pid
        pid.reset(0.5)
        n = int(round(t_end / self.dt))
        iL = 0.0
        vC = 0.0
        vin = self.vin
        r = self.r
        d = pid.d
        # delayed-duty queue: (apply_k, duty) — ponytail: list is tiny (delay ≤7 Ts)
        pending = []  # sorted by apply_k
        d_next = pid.d  # duty to apply after delay
        # decimated logs
        step = max(1, self.steps_per_cycle // log_div)
        t_log, il_log, vc_log, d_log = [], [], [], []
        dcm_periods = 0
        entered_dcm = False
        cycle_idx = 0
        ton = 0
        for k in range(n):
            t = t0 + k * self.dt
            if dist is not None:
                vin, r = dist(t, vin, r)
            # apply any delayed duty whose time has come (before cycle start)
            if pending and k >= pending[0][0]:
                # latest due duty wins
                due = [duty for ak, duty in pending if k >= ak]
                d_next = due[-1]
                pending = [(ak, duty) for ak, duty in pending if k < ak]
            # PWM: duty quantized, applied at start of each switching cycle
            if cycle_idx == 0:
                d_eff = quantize_abs(d_next, self.pwm_bits, 1.0) if not self.pwm_ideal else d_next
                d = min(D_MAX, max(D_MIN, d_eff))
                ton = round(self.steps_per_cycle * d)
                entered_dcm = False
            off = cycle_idx >= ton
            if off:
                iL += self.dt * (vC / self.l)
                if iL <= 0.0:
                    iL = 0.0  # zero-current boundary (diode reverse-blocks)
                    if not entered_dcm:
                        dcm_periods += 1
                        entered_dcm = True
                    vC += self.dt * (-vC / (r * self.c))
                else:
                    vC += self.dt * (-(iL + vC / r) / self.c)
            else:
                iL += self.dt * (vin / self.l)
                vC += self.dt * (-vC / (r * self.c))
            # controller update at sample instants
            if k % int(round(self.ts / self.dt)) == 0:
                # acquisition noise (output-frame) before quantization
                vc_noisy = vC + (np.random.randn() * self.adc_noise_sigma if self.adc_noise_sigma > 0 else 0.0)
                vmeas = quantize_abs(vc_noisy, self.adc_bits, self.adc_fs) if not self.adc_ideal else vc_noisy
                pid.step(vmeas, vref)
                # schedule duty effect after computational delay
                if self._delay_steps > 0:
                    pending.append((k + self._delay_steps, pid.d))
                else:
                    d_next = pid.d
            cycle_idx = (cycle_idx + 1) % self.steps_per_cycle
            if k % step == 0:
                t_log.append(t)
                il_log.append(iL)
                vc_log.append(vC)
                d_log.append(d)
        # ponytail: per-period cap — at most one DCM event per switching period
        n_sw = int(round(t_end * self.fsw))
        assert dcm_periods <= n_sw + 1, f"DCM {dcm_periods} > N_sw {n_sw}"
        return SimResult(np.array(t_log), np.array(il_log), np.array(vc_log),
                         np.array(d_log), dcm_periods)

    def run_continuous(self, t_end, vref, dist=None, log_div=40, t0=0.0, pid=None):
        """Run with a continuous (analog) controller updated at every dt."""
        pid = pid or ContinuousPID(self.pid.kp, self.pid.ki, self.pid.kd)
        pid.reset(0.5)
        n = int(round(t_end / self.dt))
        iL = 0.0
        vC = 0.0
        vin = self.vin
        r = self.r
        step = max(1, self.steps_per_cycle // log_div)
        t_log, il_log, vc_log, d_log = [], [], [], []
        dcm_periods = 0
        entered_dcm = False
        cycle_idx = 0
        ton = 0
        d = pid.d
        for k in range(n):
            t = t0 + k * self.dt
            if dist is not None:
                vin, r = dist(t, vin, r)
            if cycle_idx == 0:
                d = pid.d
                ton = round(self.steps_per_cycle * d)
                entered_dcm = False
            off = cycle_idx >= ton
            if off:
                iL += self.dt * (vC / self.l)
                if iL <= 0.0:
                    iL = 0.0
                    if not entered_dcm:
                        dcm_periods += 1
                        entered_dcm = True
                    vC += self.dt * (-vC / (r * self.c))
                else:
                    vC += self.dt * (-(iL + vC / r) / self.c)
            else:
                iL += self.dt * (vin / self.l)
                vC += self.dt * (-vC / (r * self.c))
            pid.step(vC, vref, self.dt)
            cycle_idx = (cycle_idx + 1) % self.steps_per_cycle
            if k % step == 0:
                t_log.append(t)
                il_log.append(iL)
                vc_log.append(vC)
                d_log.append(pid.d)
        n_sw = int(round(t_end * self.fsw))
        assert dcm_periods <= n_sw + 1, f"DCM {dcm_periods} > N_sw {n_sw}"
        return SimResult(np.array(t_log), np.array(il_log), np.array(vc_log),
                         np.array(d_log), dcm_periods)

    def run_open_loop(self, t_end, d, t_settle_frac=0.5):
        """Open-loop run at fixed duty; returns the CCM steady-state window."""
        n = int(round(t_end / self.dt))
        iL = 0.0
        vC = 0.0
        steps_per_cycle = self.steps_per_cycle
        ton = round(steps_per_cycle * d)
        last_il = np.zeros(steps_per_cycle)
        last_vc = np.zeros(steps_per_cycle)
        for k in range(n):
            off = k % steps_per_cycle >= ton
            if off:
                iL += self.dt * (vC / self.l)
                if iL <= 0.0:
                    iL = 0.0
                    vC += self.dt * (-vC / (self.r * self.c))
                else:
                    vC += self.dt * (-(iL + vC / self.r) / self.c)
            else:
                iL += self.dt * (self.vin / self.l)
                vC += self.dt * (-vC / (self.r * self.c))
            last_il[k % steps_per_cycle] = iL
            last_vc[k % steps_per_cycle] = vC
        return last_vc.mean(), last_il.mean()


class SimResult:
    def __init__(self, t, il, vc, d, dcm_cycles):
        self.t, self.il, self.vc, self.d = t, il, vc, d
        self.dcm_cycles = dcm_cycles
        self.dt = t[1] - t[0] if len(t) > 1 else 1.0

    def steady(self, frac=0.5):
        """Steady-state window = last `frac` of the run."""
        i = int(len(self.t) * (1 - frac))
        return self.t[i:], self.vc[i:]

    def vout_ss(self, frac=0.5):
        return self.steady(frac)[1]


# ---------------- metrics ----------------

def settling_time(t, y, band_frac=0.02, frac=0.5):
    """2% settling time: last crossing of band around steady mean."""
    ys = y[int(len(y) * (1 - frac)):].mean()
    band = band_frac * abs(ys)
    ok = np.abs(y - ys) > band
    if not ok.any():
        return t[0], ys
    return t[np.flatnonzero(ok)[-1]], ys


def overshoot(t, y, frac=0.5):
    ys = y[int(len(y) * (1 - frac)):].mean()
    yf = y[-1]
    # measured relative to the reference excursion (Vref is the target)
    peak = np.max(np.abs(y))
    ref = abs(yf)
    return max(0.0, (peak - ref) / ref * 100.0), ys


def steady_error(y, vref, frac=0.5):
    return y[int(len(y) * (1 - frac)):].mean() - vref


def ripple_pp(y, frac=0.5):
    ys = y[int(len(y) * (1 - frac)):]
    return ys.max() - ys.min()


def iae_itae(t, y, vref):
    e = np.abs(y - vref)
    return np.trapezoid(e, t), np.trapezoid(t * e, t)


def all_metrics(r, vref, t0=0.0, steady_frac=0.5):
    """All metrics. If t0 > 0 (disturbance runs), transient metrics are
    evaluated on the post-step window and ripple on the final steady_frac."""
    t, vc = r.t, r.vc
    if t0 > 0:
        m = t >= t0
        t, vc = t[m], vc[m]
    ts, ss = settling_time(t, vc)
    ov, _ = overshoot(t, vc)
    se = steady_error(vc, vref)
    if t0 > 0:  # ripple/limit-cycle only once transients have settled
        n = max(1, int(round(len(vc) * steady_frac)))
        pp = vc[-n:].max() - vc[-n:].min()
        iae, itae = iae_itae(t, vc, vref)
        return dict(t_settle=ts, overshoot_pct=ov, ss_error=se,
                    ripple_pp=pp, iae=iae, itae=itae,
                    limit_cycle_amp=pp, dcm_cycles=r.dcm_cycles)
    pp = ripple_pp(vc)
    iae, itae = iae_itae(t, vc, vref)
    return dict(t_settle=ts, overshoot_pct=ov, ss_error=se,
                ripple_pp=pp, iae=iae, itae=itae,
                limit_cycle_amp=pp, dcm_cycles=r.dcm_cycles)


# ---------------- self-checks ----------------

def _check_open_loop_steady_state():
    bb = BuckBoost()
    for d in (0.3, 0.5, 0.7):
        vc, il = bb.run_open_loop(0.05, d)
        v_exp = -12.0 * d / (1 - d)
        il_exp = abs(v_exp) / (bb.r * (1 - d))
        assert abs(vc - v_exp) < 0.15 * abs(v_exp) + 0.02, (d, vc, v_exp)
        assert abs(il - il_exp) < 0.15 * il_exp, (d, il, il_exp)
    print("open-loop CCM steady state: vC=-D/(1-D)Vin, iL=-vC/(R(1-D))  OK")


def _check_regulation_polarity():
    """From rest, the loop must drive Vout toward Vref = -12 V (not +12 or 0)."""
    bb = BuckBoost(pid=DigitalPID(0.005, 50.0, 0.0, 1 / bb_fsw()))
    r = bb.run(0.05, -12.0)
    vss = r.vout_ss().mean()
    assert vss < -11.0, vss
    print(f"closed-loop polarity/convergence: Vout_ss = {vss:.3f} V  OK")


def bb_fsw():
    return 100e3


def _check_dcm_boundary():
    bb = BuckBoost(r=2000.0, pid=DigitalPID(0.05, 100, 0.0, 1 / bb_fsw()))
    r = bb.run(0.05, -12.0)
    assert r.il.min() >= 0.0 - 1e-12, r.il.min()
    assert r.dcm_cycles > 0, "expected DCM at light load"
    print(f"DCM zero-current boundary: iL_min={r.il.min():.3f} A, "
          f"DCM cycles={r.dcm_cycles}  OK")


def _check_quantizer():
    q = quantize_abs(-11.937, 8, 24.0)
    expect = np.round(-11.937 / 24.0 * 256) / 256 * 24.0
    assert np.allclose(q, expect), (q, expect)
    print("ADC quantizer round-trip  OK")


def _check_metrics_synthetic():
    t = np.linspace(0, 1, 100000)
    # pure square wave settling at ts=0.4 to level 10
    y = np.where(t < 0.4, 0.0, 10.0)
    ts, ss = settling_time(t, y)
    assert abs(ts - 0.4) < 0.01, ts
    assert abs(ss - 10.0) < 1e-9
    assert ripple_pp(y, 0.5) == 0.0
    print("metrics on synthetic signals  OK")


def _check_anti_windup():
    """Velocity form must respond immediately once error returns (no windup
    hang: saturation never enters an accumulating integrator state)."""
    pid = DigitalPID(0.1, 2000, 5e-5, 1e-5)
    pid.reset(0.5)
    for _ in range(2000):
        pid.step(-2.0, -12.0)  # e = +10 -> duty pins at D_MAX
    assert pid.d >= D_MAX - 1e-9
    d_before = pid.d
    for _ in range(50):
        pid.step(-12.0, -12.0)  # e = 0 exactly: velocity form holds duty
    assert pid.d == d_before  # frozen only because error is exactly zero
    # realistic recovery: mildly over-regulated output, e = -1 -> duty must
    # fall immediately (a windup-laden positional PID would stay pegged)
    pid.step(-13.0, -12.0)
    after_neg = pid.d
    assert after_neg < d_before - 0.05, after_neg
    pid.step(-11.0, -12.0)
    assert pid.d > after_neg, pid.d  # e = +1 -> duty rises again
    print("velocity-form saturation recovery  OK")


def self_check():
    _check_open_loop_steady_state()
    _check_quantizer()
    _check_metrics_synthetic()
    _check_regulation_polarity()
    _check_dcm_boundary()
    _check_anti_windup()
    print("ALL SELF-CHECKS PASSED")


if __name__ == "__main__":
    self_check()