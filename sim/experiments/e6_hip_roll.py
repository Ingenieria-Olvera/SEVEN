"""Experiment 6a: hip roll, lean and stance width (the static half of e6 carve/roll).

Hip roll does two jobs (see hip_roll_pose() in models/build_mjcf.py):
  lean    both legs tilt the same way: the body stays level and slides sideways
          over the wheels (the legs + body + floor form a parallelogram).
          This is the pose you'd hold while carving a turn.
  spread  legs tilt opposite ways: wide (+) or narrow (−) stance.

The body rides a test rig (build base="roll"): it can slide up/down, slide
sideways and roll, but can't pitch or drive forward, so no balance controller
is needed yet. Angles ramp slowly, so the torques are ~static holding torques.
Every step goes through the torque–speed clamp in sim/actuators.py.

  .\\run sim\\experiments\\e6_hip_roll.py            # headless: summary + results/e6_hip_roll/<time>/
  .\\run sim\\experiments\\e6_hip_roll.py --view     # watch it

Hand calc (CLAUDE.md §2): τ ≈ F · h · sin φ, F = weight per leg, h = hip height.
The sim also sees the wheel's 62 mm outboard offset from the hip, which adds
F · d · cos φ on top: that's why it disagrees with the simple formula.
"""
import argparse
import datetime
import math
import shutil
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "models"))
sys.path.insert(0, str(ROOT / "sim"))
from build_mjcf import G, PARAMS, build, hip_roll_pose, leg_pose, load_params   # noqa: E402
from actuators import load_actuators                                            # noqa: E402

LEG = [f"{j}_{s}" for s in "LR" for j in ("hip_roll", "hip_pitch", "knee")]
HEAD_HOLD = {"neck_pan": (20.0, 0.5), "head_tilt": (20.0, 0.5), "eye_tilt": (0.5, 0.01)}
LEG_GAINS = (300.0, 6.0)     # kp N·m/rad, kd N·m·s/rad: stiff enough to hold within ~1–2°

# (lean_deg, spread_deg, ramp_s, hold_s): move to this target over ramp_s, then hold.
SCRIPT = [
    (0, 0, 0.0, 0.5),        # settle
    (15, 0, 1.0, 0.5),       # lean left
    (-15, 0, 2.0, 0.5),      # lean right
    (0, 0, 1.0, 0.3),
    (0, 15, 1.0, 0.5),       # wide stance
    (0, -10, 2.0, 0.5),      # narrow stance (−15 would put the wheels on top of each other)
    (0, 0, 1.0, 0.3),
]


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def roll_targets(t):
    """(lean, spread) in degrees at time t, following SCRIPT."""
    prev, t0 = (0.0, 0.0), 0.0
    for lean, spread, ramp, hold in SCRIPT:
        if t < t0 + ramp + hold:
            s = smoothstep((t - t0) / ramp) if ramp > 0 else 1.0
            return prev[0] + s * (lean - prev[0]), prev[1] + s * (spread - prev[1])
        prev, t0 = (lean, spread), t0 + ramp + hold
    return prev


def script_length():
    return sum(r + h for _, _, r, h in SCRIPT)


class RollTest:
    def __init__(self, p, theta_deg):
        self.p = p
        self.m = mujoco.MjModel.from_xml_string(build(p, base="roll"))
        self.d = mujoco.MjData(self.m)
        m, d = self.m, self.d
        mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
        self.theta = math.radians(theta_deg)
        self.acts = load_actuators(p)
        self.ctrl_every = max(1, round(1 / (p["sim"]["control_hz"] * m.opt.timestep)))
        self.steps = 0
        self.log = []
        self.tau_cmd = {n: 0.0 for n in LEG}

        # Start already in the test pose so there's no drop at t=0
        for n, q in self.targets(0.0).items():
            d.joint(n).qpos[0] = q
        mujoco.mj_forward(m, d)
        lowest = min(d.geom_xpos[m.geom(w).id][2] for w in ("wheel_L", "wheel_R")) - p["robot"]["wheel_radius"]
        d.joint("rig_z").qpos[0] -= lowest                       # put the wheels on the floor

        self.j = [(n, m.joint(n).qposadr[0], m.joint(n).dofadr[0], m.actuator(n).id, self.acts[n.rsplit("_", 1)[0]])
                  for n in LEG]
        self.head = []
        for a, gains in HEAD_HOLD.items():
            ai = m.actuator(a).id
            jid = m.actuator_trnid[ai, 0]
            self.head.append((m.jnt_qposadr[jid], m.jnt_dofadr[jid], ai, self.acts[a], gains))
        self.wheels = [(m.joint(f"wheel_{s}").dofadr[0], m.actuator(f"wheel_{s}").id) for s in "LR"]

    def targets(self, t):
        lean, spread = roll_targets(t)
        roll = hip_roll_pose(math.radians(lean), math.radians(spread))
        out = {}
        for s in "LR":
            out.update({f"{j}_{s}": q for j, q in leg_pose(self.theta, roll[s]).items()})
        return out

    def step(self):
        m, d = self.m, self.d
        control_tick = self.steps % self.ctrl_every == 0
        if control_tick:
            tgt = self.targets(d.time)
            kp, kd = LEG_GAINS
            for n, qa, va, _, _ in self.j:
                self.tau_cmd[n] = kp * (tgt[n] - d.qpos[qa]) - kd * d.qvel[va]

        row = None
        if control_tick:
            lean, spread = roll_targets(d.time)
            row = {"t": d.time, "lean_cmd": lean, "spread_cmd": spread,
                   "body_y": d.joint("rig_y").qpos[0], "body_roll": math.degrees(d.joint("rig_roll").qpos[0])}
        for n, qa, va, ai, act in self.j:
            w = d.qvel[va]
            tau = float(np.clip(act.limit(self.tau_cmd[n], w), *m.actuator_ctrlrange[ai]))
            d.ctrl[ai] = tau
            if row is not None:
                row.update({f"{n}_cmd": self.tau_cmd[n], f"{n}_tau": tau, f"{n}_w": w,
                            f"{n}_q": math.degrees(d.qpos[qa]), f"{n}_amps": act.current(tau)})
        for qa, va, ai, act, (kp, kd) in self.head:
            w = d.qvel[va]
            d.ctrl[ai] = float(np.clip(act.limit(-kp * d.qpos[qa] - kd * w, w), *m.actuator_ctrlrange[ai]))
        for va, ai in self.wheels:                                 # wheels: just damp any spin
            d.ctrl[ai] = -0.5 * d.qvel[va]

        mujoco.mj_step(m, d)
        self.steps += 1
        if row is not None:
            self.log.append(row)


def hand_calc(p, M, theta):
    """Hand-calc hip-roll torque per leg vs lean angle (deg): simple and with the wheel offset."""
    r = p["robot"]
    h = (r["link_femur"] + r["link_shin"]) * math.cos(theta) + r["wheel_radius"]   # hip height above floor
    F = M * G / 2
    dy = r["wheel_offset_y"]                                                       # wheel outboard of hip

    def simple(phi_deg):
        return F * h * math.sin(math.radians(abs(phi_deg)))

    def with_offset(phi_deg):
        # outer leg (wheel swung outward) gets offset + tilt; worst leg
        phi = math.radians(abs(phi_deg))
        return F * (dy * math.cos(phi) + h * math.sin(phi))
    return h, F, simple, with_offset


def run(p, theta_deg, view=False, slow=0.5):
    test = RollTest(p, theta_deg)
    t_end = script_length()
    if view:
        with mujoco.viewer.launch_passive(test.m, test.d) as viewer:
            while viewer.is_running() and test.d.time < t_end:
                t0 = time.perf_counter()
                test.step()
                viewer.sync()
                dt = test.m.opt.timestep / slow - (time.perf_counter() - t0)
                if dt > 0:
                    time.sleep(dt)
    else:
        while test.d.time < t_end:
            test.step()
    return test, pd.DataFrame(test.log)


def peaks(df, mask):
    """Peak |τ|, peak |cmd|, peak current over the rows in mask, worst of L/R."""
    sub = df[mask]
    tau = max(sub[f"hip_roll_{s}_tau"].abs().max() for s in "LR")
    cmd = max(sub[f"hip_roll_{s}_cmd"].abs().max() for s in "LR")
    amps = max(sub[f"hip_roll_{s}_amps"].max() for s in "LR")
    err = max((sub[f"hip_roll_{s}_q"] - sub[f"hip_roll_{s}_target"]).abs().max() for s in "LR")
    return tau, cmd, amps, err


def add_targets(df):
    for s in "LR":
        df[f"hip_roll_{s}_target"] = [math.degrees(hip_roll_pose(math.radians(a), math.radians(b))[s])
                                      for a, b in zip(df.lean_cmd, df.spread_cmd)]
    return df


def summarize(p, test, df, theta_deg):
    a = test.acts["hip_roll"]
    cfg = p["actuators"]["hip_roll"]
    M = mujoco.mj_getTotalmass(test.m)
    h, F, simple, with_offset = hand_calc(p, M, math.radians(theta_deg))
    L1L2 = p["robot"]["link_femur"] + p["robot"]["link_shin"]
    lines = [f"θ = {theta_deg}°   mass {M:.2f} kg   hip height {h:.3f} m   load per leg {F:.1f} N",
             f"hip_roll: {cfg['motor']} × {cfg['ratio']}:1   τ_peak {a.tau_peak:.1f}   τ_cont {a.tau_cont:.1f} N·m",
             "",
             f"{'move':<22}{'sim τ':>8}{'hand':>8}{'hand+off':>9}{'%cont':>7}{'%peak':>7}{'amps':>7}{'err°':>6}"]
    lean_mag = df.lean_cmd.abs()
    spread = df.spread_cmd
    cases = [("settle (0°)", (lean_mag < 0.01) & (spread.abs() < 0.01) & (df.t < 0.5), 0),
             ("lean ±15°", lean_mag > 14.9, 15),
             ("wide +15°", spread > 14.9, 15),
             ("narrow −10°", spread < -9.9, 10)]
    out = {}
    for name, mask, phi in cases:
        tau, cmd, amps, err = peaks(df, mask)
        lines.append(f"{name:<22}{tau:8.2f}{simple(phi):8.2f}{with_offset(phi):9.2f}"
                     f"{tau / a.tau_cont * 100:7.0f}{tau / a.tau_peak * 100:7.0f}{amps:7.1f}{err:6.1f}")
        out[name] = tau
    lines.append("")
    lines.append(f"peak over the whole run (incl. ramps): {peaks(df, df.t >= 0)[0]:.2f} N·m, "
                 f"saturated: {'YES' if peaks(df, df.t >= 0)[1] > peaks(df, df.t >= 0)[0] + 1e-6 else 'no'}")
    lean_rows = df[lean_mag > 14.9]
    lines.append(f"body sideways shift at 15° lean: {lean_rows.body_y.abs().max() * 100:.1f} cm "
                 f"(≈ h_leg·sin15° = {L1L2 * math.cos(math.radians(theta_deg)) * math.sin(math.radians(15)) * 100:.1f} cm), "
                 f"body roll {lean_rows.body_roll.abs().max():.1f}°")
    lines.append("  (not a true parallelogram: each wheel sits outboard of its leg, so tilting both legs raises")
    lines.append("   one wheel and lowers the other relative to the hips → the body rolls a few degrees too)")
    track0 = 2 * (p["robot"]["hip_y"] + p["robot"]["wheel_offset_y"])
    for phi in (10, 15):
        tr = track0 - 2 * L1L2 * math.cos(math.radians(theta_deg)) * math.sin(math.radians(phi))
        lines.append(f"track at narrow −{phi}°: {tr * 100:.1f} cm"
                     + ("  ← wheels would hit each other (wheel width " f"{p['robot']['wheel_width'] * 100:.0f} cm)"
                        if tr < p["robot"]["wheel_width"] + 0.02 else ""))
    lines += ["hand = F·h·sinφ (CLAUDE.md §2); hand+off = F·(d·cosφ + h·sinφ), d = wheel offset from hip.",
              "Both assume each wheel carries half the weight and can slide sideways freely. In the sim the",
              "outer wheel takes most of the load almost under its hip (lean is cheaper than the hand calc),",
              "and stance changes must drag the loaded wheels sideways: friction ≈ μ·F·h, with μ = contact.",
              "wheel_friction[0] (ESTIMATE). Narrow/wide numbers are mostly that scrub, so change stance slowly",
              "or while a leg is unloaded."]
    return "\n".join(lines), out


def plot(results, path):
    """|torque| vs angle, one row per θ: lean on the left, stance width on the right."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    series = {"L": "#2a78d6", "R": "#eb6834"}                 # categorical slots 1–2
    ink, muted, grid, limit = "#2b2b2a", "#6b6a64", "#e4e3dd", "#e34948"
    t_lean_end = sum(r + h for _, _, r, h in SCRIPT[:4])      # SCRIPT rows 1–4 = lean, rest = stance
    fig, ax = plt.subplots(len(results), 2, figsize=(11, 4 * len(results)), squeeze=False)
    for row, (theta_deg, test, df) in enumerate(results):
        a = test.acts["hip_roll"]
        M = mujoco.mj_getTotalmass(test.m)
        _, _, simple, with_offset = hand_calc(test.p, M, math.radians(theta_deg))
        for col, (key, label, mask) in enumerate((
                ("lean_cmd", "lean (+ = left), deg", (df.t > 0.5) & (df.t < t_lean_end)),
                ("spread_cmd", "stance (+ = wide, − = narrow), deg", df.t >= t_lean_end))):
            sub, axis = df[mask], ax[row, col]
            for s, c in series.items():
                axis.plot(sub[key], sub[f"hip_roll_{s}_tau"].abs(), color=c, lw=2, label=f"hip_roll_{s} (sim)")
                sat = sub[f"hip_roll_{s}_cmd"].abs() > sub[f"hip_roll_{s}_tau"].abs() + 0.05
                if sat.any():
                    axis.plot(sub[key][sat], sub[f"hip_roll_{s}_cmd"][sat].abs().clip(upper=1.1 * a.tau_peak), ".",
                              color=c, ms=2, label=f"hip_roll_{s} wanted (saturated)")
            xs = np.linspace(sub[key].min(), sub[key].max(), 50)
            axis.plot(xs, [with_offset(x) for x in xs], "--", color=ink, lw=0.8, label="hand calc incl. wheel offset")
            axis.plot(xs, [simple(x) for x in xs], ":", color=ink, lw=0.8, label="hand calc F·h·sinφ")
            for lim, ls, name in ((a.tau_cont, "-.", "τ_cont"), (a.tau_peak, "-", "τ_peak")):
                axis.axhline(lim, color=limit, lw=0.8, ls=ls)
                axis.text(0.01, lim + 0.4, name, color=muted, fontsize=7, transform=axis.get_yaxis_transform())
            axis.set_ylim(0, 1.15 * a.tau_peak)
            axis.set_xlabel(label, color=muted, fontsize=8)
            axis.set_ylabel("|hip roll torque|, N·m", color=muted, fontsize=8)
            axis.set_title(f"θ = {theta_deg}°, {M:.1f} kg", color=ink, fontsize=9)
            axis.legend(fontsize=6, frameon=False, loc="upper center")
            axis.grid(color=grid, lw=0.6); axis.set_axisbelow(True)
            for sp in ("top", "right"):
                axis.spines[sp].set_visible(False)
            axis.tick_params(colors=muted, labelsize=7)
    fig.suptitle("Hip roll holding torque. Stance changes scrub the wheels sideways, so they loop (friction).",
                 color=ink, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor="white")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="watch the first pose in the viewer")
    ap.add_argument("--slow", type=float, default=0.5, help="viewer speed (1 = real time)")
    ap.add_argument("--theta", type=float, nargs="+", help="thigh angles to test, deg (default: stand and 55° crouch)")
    args = ap.parse_args()

    p = load_params()
    thetas = args.theta or [p["robot"]["stand_theta_deg"], 55]
    if args.view:
        test, df = run(p, thetas[0], view=True, slow=args.slow)
        print(summarize(p, test, add_targets(df), thetas[0])[0])
        return

    out = ROOT / "results" / "e6_hip_roll" / datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    results, texts = [], []
    for th in thetas:
        test, df = run(p, th)
        df = add_targets(df)
        text, _ = summarize(p, test, df, th)
        print(text + "\n")
        texts.append(text)
        results.append((th, test, df))
        df.to_csv(out / f"roll_theta{th:g}.csv", index=False)
    shutil.copy(PARAMS, out / "params.yaml")
    (out / "summary.txt").write_text("\n\n".join(texts), encoding="utf-8")
    plot(results, out / "hip_roll.png")
    print(f"saved {out.relative_to(ROOT)}  (hip_roll.png, roll_theta*.csv, summary.txt, params.yaml)")


if __name__ == "__main__":
    main()
