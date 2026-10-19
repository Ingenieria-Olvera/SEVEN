"""Experiment 4: jump.

Crouch → max-effort push-off → tuck in flight → land, with the torque–speed
limit applied every physics step. Everything tunable is in config/params.yaml
(robot.mass_scale, actuators.*, motors.*, jump.*).

  .\\run sim\\experiments\\e4_jump.py              # headless: prints results, saves CSV + plot
  .\\run sim\\experiments\\e4_jump.py --view       # watch it (slow motion)
  .\\run sim\\experiments\\e4_jump.py --view --slow 1   # real time

Results land in results/e4_jump/<date_time>/ with a copy of params.yaml, so
every run can be traced back to the exact numbers that produced it.

By default the body rides a vertical test rig (jump.base: vertical): it can
slide up and down but not tip, so this measures what the legs can do without
needing a balance controller yet.
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
from build_mjcf import PARAMS, build, leg_pose, load_params   # noqa: E402
from actuators import load_actuators                          # noqa: E402

JOINTS = [f"{j}_{s}" for s in "LR" for j in ("hip_roll", "hip_pitch", "knee")]
# Head actuators just hold 0 during the jump: {actuator name: (kp, kd)}
HEAD_HOLD = {"neck_pan": (20.0, 0.5), "head_tilt": (20.0, 0.5), "eye_tilt": (0.5, 0.01)}


def pose(theta):
    """Leg joint targets for thigh angle theta (rad): keeps the wheel under the hip."""
    return leg_pose(theta)


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class Jump:
    def __init__(self, p, model, data):
        self.p, self.m, self.d = p, model, data
        self.jp = p["jump"]
        self.acts = load_actuators(p)
        self.r_wheel = p["robot"]["wheel_radius"]
        rad = math.radians
        self.th_stand, self.th_crouch = rad(p["robot"]["stand_theta_deg"]), rad(self.jp["crouch_deg"])
        self.th_ext, self.th_tuck, self.th_land = rad(self.jp["extend_deg"]), rad(self.jp["tuck_deg"]), rad(self.jp["land_deg"])
        self.ctrl_every = max(1, round(1 / (p["sim"]["control_hz"] * model.opt.timestep)))

        # Per joint: qpos index, qvel index, actuator index, actuator model
        self.j = [(n, model.joint(n).qposadr[0], model.joint(n).dofadr[0], model.actuator(n).id,
                   self.acts[n.rsplit("_", 1)[0]]) for n in JOINTS]
        self.head = []
        for a, gains in HEAD_HOLD.items():
            ai = model.actuator(a).id
            jid = model.actuator_trnid[ai, 0]                 # the joint this actuator drives
            self.head.append((a, model.jnt_qposadr[jid], model.jnt_dofadr[jid], ai, self.acts[a], gains))
        self.floor = model.geom("floor").id
        self.wheels = {model.geom("wheel_L").id, model.geom("wheel_R").id}

        self.phase, self.t_phase = "settle", 0.0
        self.no_contact_steps = 0
        self.tau_cmd = {n: 0.0 for n in JOINTS}
        self.log = []
        self.steps = 0
        self.flight_clearance = 0.0
        self.z_stand = None
        self.z_apex = -1.0
        self.takeoff_t = self.land_t = None

    def wheels_touching(self):
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if self.floor in (c.geom1, c.geom2) and ({c.geom1, c.geom2} & self.wheels):
                return True
        return False

    def set_phase(self, name):
        self.phase, self.t_phase = name, self.d.time

    def targets_and_gains(self):
        """State machine: which pose to aim for and how hard."""
        jp, t, d = self.jp, self.d.time, self.d
        g = jp["gains"]
        touching = self.wheels_touching()
        self.no_contact_steps = 0 if touching else self.no_contact_steps + self.ctrl_every

        if self.phase == "settle" and t >= jp["t_settle"]:
            self.z_stand = d.body("torso").xpos[2]
            self.set_phase("crouch")
        elif self.phase == "crouch" and t - self.t_phase >= jp["t_crouch"]:
            self.set_phase("hold")
        elif self.phase == "hold" and t - self.t_phase >= jp["t_hold"]:
            self.set_phase("push")
        elif self.phase == "push":
            if self.no_contact_steps >= 10:                      # 10 ms with no wheel contact = airborne
                self.takeoff_t = t
                self.set_phase("flight")
            elif t - self.t_phase > 0.6:                          # never left the ground
                self.set_phase("land")
        elif self.phase == "flight" and touching:
            self.land_t = t
            self.set_phase("land")

        if self.phase == "settle":
            return pose(self.th_stand), g["pose"]
        if self.phase == "crouch":
            s = smoothstep((t - self.t_phase) / jp["t_crouch"])
            return pose(self.th_stand + s * (self.th_crouch - self.th_stand)), g["pose"]
        if self.phase == "hold":
            return pose(self.th_crouch), g["pose"]
        if self.phase == "push":
            return pose(self.th_ext), g["push"]
        if self.phase == "flight":
            return pose(self.th_tuck), g["pose"]
        return pose(self.th_land), g["land"]

    def step(self):
        """One physics step: controller at control_hz, torque–speed clamp every step."""
        m, d = self.m, self.d
        control_tick = self.steps % self.ctrl_every == 0
        if control_tick:
            tgt, (kp, kd) = self.targets_and_gains()
            for n, qa, va, _, _ in self.j:
                jt, side = n.rsplit("_", 1)
                if self.phase == "push" and jt == "hip_pitch":
                    # Hip steers, knee lifts: track ½ × the knee's *actual* angle so the
                    # wheel stays under the hip while the knee extends at max effort.
                    kp_h, kd_h = self.jp["gains"]["push_hip"]
                    q_knee = d.qpos[self.m.joint(f"knee_{side}").qposadr[0]]
                    self.tau_cmd[n] = kp_h * (0.5 * q_knee - d.qpos[qa]) - kd_h * d.qvel[va]
                else:
                    self.tau_cmd[n] = kp * (tgt[jt] - d.qpos[qa]) - kd * d.qvel[va]

        row = {"t": d.time, "phase": self.phase} if control_tick else None
        for n, qa, va, ai, act in self.j:
            w = d.qvel[va]
            tau = act.limit(self.tau_cmd[n], w)
            tau = float(np.clip(tau, *m.actuator_ctrlrange[ai]))
            d.ctrl[ai] = tau
            if row is not None:
                row.update({f"{n}_cmd": self.tau_cmd[n], f"{n}_tau": tau, f"{n}_w": w,
                            f"{n}_q": d.qpos[qa], f"{n}_amps": act.current(tau)})
        for n, qa, va, ai, act, (kp, kd) in self.head:
            w = d.qvel[va]
            d.ctrl[ai] = float(np.clip(act.limit(-kp * d.qpos[qa] - kd * w, w), *m.actuator_ctrlrange[ai]))

        mujoco.mj_step(m, d)
        self.steps += 1

        wheel_low = min(d.body("wheel_L").xpos[2], d.body("wheel_R").xpos[2]) - self.r_wheel
        torso_z = d.body("torso").xpos[2]
        if self.phase == "flight":
            self.flight_clearance = max(self.flight_clearance, wheel_low)
            self.z_apex = max(self.z_apex, torso_z)
        if row is not None:
            row.update({"torso_z": torso_z, "wheel_clearance": wheel_low})
            self.log.append(row)


def summarize(p, jump, df):
    acts = jump.acts
    push = df[df.phase == "push"]
    after = df[df.phase.isin(["flight", "land"])]
    lines = []
    M = mujoco.mj_getTotalmass(jump.m)
    lines.append(f"mass {M:.2f} kg (mass_scale {p['robot']['mass_scale']}), base = {p['jump']['base']}, "
                 f"crouch {p['jump']['crouch_deg']}° → extend {p['jump']['extend_deg']}°")
    for jt in ("hip_pitch", "knee"):
        a, cfg = acts[jt], p["actuators"][jt]
        lines.append(f"{jt:<9}: {cfg['motor']} × {cfg['ratio']}:1  τ_peak {a.tau_peak:5.1f} N·m  ω0 {a.w0:5.1f} rad/s")
    lines.append("")
    if jump.takeoff_t is None:
        lines.append("RESULT: NO TAKEOFF. The legs could not launch this mass.")
    else:
        lines.append(f"RESULT: takeoff at t={jump.takeoff_t:.3f} s, push lasted {jump.takeoff_t - push.t.iloc[0]:.3f} s")
        lines.append(f"  wheel clearance (lowest wheel bottom above floor): {jump.flight_clearance * 100:5.1f} cm")
        lines.append(f"  body rise above standing height:                  {(jump.z_apex - jump.z_stand) * 100:5.1f} cm")
    lines.append("")
    lines.append(f"{'joint':<12}{'peak cmd':>9}{'peak τ':>8}{'sat %':>7}{'peak ω':>8}{'ω0':>6}{'peak A':>8}{'i_peak':>7}   (push phase; landing peaks after /)")
    for n in ("hip_pitch_L", "knee_L"):
        jt = n.rsplit("_", 1)[0]
        a = acts[jt]
        sat = (push[f"{n}_cmd"].abs() > push[f"{n}_tau"].abs() + 1e-6).mean() * 100 if len(push) else 0
        land_tau = after[f"{n}_tau"].abs().max() if len(after) else 0
        lines.append(f"{n:<12}{push[f'{n}_cmd'].abs().max():9.1f}{push[f'{n}_tau'].abs().max():8.1f}{sat:7.0f}"
                     f"{push[f'{n}_w'].abs().max():8.1f}{a.w0:6.1f}{push[f'{n}_amps'].max():8.1f}{a.i_peak:7.0f}"
                     f"   / land τ {land_tau:.1f} N·m")
    return "\n".join(lines)


def plot(df, jump, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for n, c in (("knee_L", "C0"), ("hip_pitch_L", "C1")):
        ax[0].plot(df.t, df[f"{n}_cmd"].clip(-60, 60), c, ls=":", lw=1, label=f"{n} wanted")
        ax[0].plot(df.t, df[f"{n}_tau"], c, lw=1.5, label=f"{n} got")
        ax[1].plot(df.t, df[f"{n}_w"], c, label=n)
    a = jump.acts["knee"]
    ax[0].axhline(a.tau_peak, color="gray", lw=0.8); ax[0].axhline(-a.tau_peak, color="gray", lw=0.8)
    ax[0].set_ylabel("torque N·m"); ax[0].legend(fontsize=7, ncol=2)
    ax[0].set_title("dotted = what the controller wanted, solid = what the motor could give")
    ax[1].axhline(a.w0, color="gray", lw=0.8); ax[1].axhline(-a.w0, color="gray", lw=0.8)
    ax[1].set_ylabel("joint speed rad/s (grey = ω0)"); ax[1].legend(fontsize=7)
    ax[2].plot(df.t, df.wheel_clearance * 100, label="lowest wheel bottom")
    ax[2].plot(df.t, (df.torso_z - df.torso_z.iloc[0]) * 100, label="body height change")
    ax[2].set_ylabel("cm"); ax[2].set_xlabel("time s"); ax[2].legend(fontsize=7)
    for axis in ax:
        for ph in ("crouch", "push", "flight", "land"):
            rows = df[df.phase == ph]
            if len(rows):
                axis.axvline(rows.t.iloc[0], color="k", lw=0.5, alpha=0.3)
    ax[2].text(0.01, 0.95, "vertical lines: crouch | push | flight | land", transform=ax[2].transAxes, fontsize=7, va="top")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="open the viewer")
    ap.add_argument("--slow", type=float, default=0.25, help="viewer speed (1 = real time)")
    args = ap.parse_args()

    p = load_params()
    model = mujoco.MjModel.from_xml_string(build(p, base=p["jump"]["base"]))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    jump = Jump(p, model, data)
    t_end = p["jump"]["t_total"]

    if args.view:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                t0 = time.perf_counter()
                jump.step()
                viewer.sync()
                if data.time >= t_end and jump.phase == "land" and len(jump.log) and "done" not in jump.__dict__:
                    jump.done = True
                    print(summarize(p, jump, pd.DataFrame(jump.log)))
                dt = model.opt.timestep / args.slow - (time.perf_counter() - t0)
                if dt > 0:
                    time.sleep(dt)
        return

    while data.time < t_end:
        jump.step()
    df = pd.DataFrame(jump.log)
    text = summarize(p, jump, df)
    print(text)

    out = ROOT / "results" / "e4_jump" / datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy(PARAMS, out / "params.yaml")
    df.to_csv(out / "jump.csv", index=False)
    (out / "summary.txt").write_text(text, encoding="utf-8")
    plot(df, jump, out / "jump.png")
    print(f"\nsaved {out.relative_to(ROOT)}  (jump.csv, jump.png, summary.txt, params.yaml)")


if __name__ == "__main__":
    main()
