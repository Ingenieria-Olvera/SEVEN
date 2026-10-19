"""Motor sweep: which leg motor + gear ratio works, and up to what robot mass?

For every (leg_motor × ratio × mass_scale) in params.yaml → sweep (plus extra
mass steps so we can find the limit), runs:
  1. e4 jump (vertical rig)  → wheel clearance, knee torque to hold stand and crouch
  2. e6 hip roll, lean 15°   → hip roll holding torque
  3. one-leg stand θ=30°/45° → knee torque with the whole robot on one leg (stretch goal;
     vertical rig, so only the vertical load is tested; side-to-side balance is e7)

The leg motor + ratio replace hip_roll, hip_pitch and knee. Leg actuator mass follows
the motor (motor + sweep.gearbox_mass), so a heavier motor makes the robot heavier.

Pass rules (the "goal"):
  sustained holds (2-leg stand, 2-leg crouch, 15° lean) ≤ τ_cont   → can sit there without overheating
  jump wheel clearance ≥ target (20 / 30 / 40 cm, target not decided yet, so all three are reported)
  one-leg stand: knee ≤ τ_peak, not saturated, sags < 10° (short hold, stretch goal)

  .\\run sim\\experiments\\sweep_motors.py              # full sweep (~1–2 min on 16 cores)
  .\\run sim\\experiments\\sweep_motors.py --quick      # params.yaml mass_scale list only

Results: results/sweep_motors/<time>/ → summary.md, sweep.csv, sweep.png, params.yaml
"""
import argparse
import copy
import datetime
import math
import shutil
import sys
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "models"))
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "sim" / "experiments"))
from build_mjcf import PARAMS, build, leg_pose, load_params   # noqa: E402
from actuators import load_actuators                          # noqa: E402
import e4_jump                                                # noqa: E402
import e6_hip_roll                                            # noqa: E402

LEG_JOINTS = ("hip_roll", "hip_pitch", "knee")
JUMP_TARGETS_CM = (20, 30, 40)
EXTRA_MASS_SCALES = [0.8, 0.9, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]   # added to sweep.mass_scale to find the limit


def configure(p, motor, ratio, mass_scale):
    """Copy of params with the leg motor/ratio/mass swapped in."""
    q = copy.deepcopy(p)
    eta = q["sweep"]["eta_by_ratio"][ratio]
    for j in LEG_JOINTS:
        belt = 0.02 if j == "knee" else 0.0                  # knee loses ~2% more in the belt
        q["actuators"][j].update(motor=motor, ratio=ratio, eta=eta - belt)
    q["robot"]["mass_scale"] = mass_scale
    return q


def run_jump(q):
    model = mujoco.MjModel.from_xml_string(build(q, base="vertical"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    jump = e4_jump.Jump(q, model, data)
    while data.time < q["jump"]["t_total"]:
        jump.step()
    df = pd.DataFrame(jump.log)
    settle = df[(df.phase == "settle") & (df.t > q["jump"]["t_settle"] - 0.2)]
    hold = df[df.phase == "hold"]
    push = df[df.phase == "push"]
    land = df[df.phase.isin(["flight", "land"])]
    return {
        "mass_kg": mujoco.mj_getTotalmass(model),
        "clearance_cm": jump.flight_clearance * 100 if jump.takeoff_t else 0.0,
        "body_rise_cm": (jump.z_apex - jump.z_stand) * 100 if jump.takeoff_t else 0.0,   # rest of clearance = tuck
        "stand_knee": settle.knee_L_tau.abs().mean(),
        "crouch_knee": hold.knee_L_tau.abs().mean(),
        "crouch_hip": hold.hip_pitch_L_tau.abs().mean(),
        "push_peak_amps": push.knee_L_amps.max() if len(push) else np.nan,
        "land_peak_knee": land.knee_L_tau.abs().max() if len(land) else np.nan,
    }


def run_lean(q):
    test = e6_hip_roll.RollTest(q, q["robot"]["stand_theta_deg"])
    while test.d.time < 2.0:                                 # settle + ramp to 15° lean + hold
        test.step()
    df = pd.DataFrame(test.log)
    hold = df[df.lean_cmd > 14.9]
    return {"lean_roll": max(hold[f"hip_roll_{s}_tau"].abs().max() for s in "LR")}


def run_one_leg(q, theta_deg, t_end=1.5):
    """Left leg holds theta, right leg tucked (wheel lifted). Returns knee torque and tracking error."""
    model = mujoco.MjModel.from_xml_string(build(q, base="vertical"))
    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, model.key("stand").id)
    acts = load_actuators(q)
    tgt = {f"{j}_L": v for j, v in leg_pose(math.radians(theta_deg)).items()}
    tgt.update({f"{j}_R": v for j, v in leg_pose(math.radians(75)).items()})   # tucked: wheel ~30 cm up
    for n in ("hip_pitch_L", "knee_L"):
        d.joint(n).qpos[0] = tgt[n]
    mujoco.mj_forward(model, d)
    d.joint("rig_z").qpos[0] -= d.body("wheel_L").xpos[2] - q["robot"]["wheel_radius"]   # left wheel on the floor
    j = [(n, model.joint(n).qposadr[0], model.joint(n).dofadr[0], model.actuator(n).id, acts[n.rsplit("_", 1)[0]])
         for n in tgt]
    kp, kd = 300.0, 6.0
    knee, err, sat = [], [], False
    while d.time < t_end:
        for n, qa, va, ai, act in j:
            cmd = kp * (tgt[n] - d.qpos[qa]) - kd * d.qvel[va]
            tau = float(np.clip(act.limit(cmd, d.qvel[va]), *model.actuator_ctrlrange[ai]))
            d.ctrl[ai] = tau
            if n == "knee_L" and d.time > t_end - 0.3:
                knee.append(abs(tau))
                err.append(abs(math.degrees(tgt[n] - d.qpos[qa])))
                sat |= abs(cmd) > abs(tau) + 1e-6
        mujoco.mj_step(model, d)
    return {f"oneleg{theta_deg}_knee": float(np.mean(knee)), f"oneleg{theta_deg}_err": float(np.max(err)),
            f"oneleg{theta_deg}_sat": sat}


def run_case(args):
    p, motor, ratio, ms = args
    q = configure(p, motor, ratio, ms)
    a = load_actuators(q)
    row = {"motor": motor, "ratio": ratio, "mass_scale": ms,
           "act_mass": q["motors"][motor]["mass"] + q["sweep"]["gearbox_mass"],
           "tau_peak": a["knee"].tau_peak, "tau_cont": a["knee"].tau_cont, "w0": a["knee"].w0,
           "roll_tau_cont": a["hip_roll"].tau_cont}
    row.update(run_jump(q))
    row.update(run_lean(q))
    row.update(run_one_leg(q, 30))
    row.update(run_one_leg(q, 45))
    return row


def judge(df):
    df["hold_ok"] = ((df.stand_knee <= df.tau_cont) & (df.crouch_knee <= df.tau_cont)
                     & (df.lean_roll <= df.roll_tau_cont))
    for th in (30, 45):
        df[f"oneleg{th}_ok"] = (df[f"oneleg{th}_knee"] <= df.tau_peak) & (df[f"oneleg{th}_err"] < 10.0) & ~df[f"oneleg{th}_sat"]
    for cm in JUMP_TARGETS_CM:
        df[f"jump{cm}_ok"] = df.hold_ok & (df.clearance_cm >= cm)
    return df


def max_mass(sub, col):
    """Heaviest mass that passes `col`, only counting masses below the first failure
    (so a lucky pass above a failure doesn't count)."""
    best = None
    for _, r in sub.sort_values("mass_kg").iterrows():
        if not r[col]:
            break
        best = r.mass_kg
    return best


def summary_table(df, p):
    wheel = load_actuators(p)["wheel"]
    rw = p["robot"]["wheel_radius"]
    rows = []
    for (motor, ratio), sub in df.groupby(["motor", "ratio"], sort=False):
        r0 = sub.iloc[0]
        rows.append({
            "motor": motor, "ratio": ratio, "act kg": r0.act_mass,
            "τ_peak": r0.tau_peak, "τ_cont": r0.tau_cont, "ω0": r0.w0,
            **{f"max kg @ jump ≥{cm} cm": max_mass(sub, f"jump{cm}_ok") for cm in JUMP_TARGETS_CM},
            "max kg holds only": max_mass(sub, "hold_ok"),
            "max kg 1-leg 30°": max_mass(sub, "oneleg30_ok"),
            "max kg 1-leg 45°": max_mass(sub, "oneleg45_ok"),
        })
    t = pd.DataFrame(rows)

    def fmt(v):
        if v is None or (isinstance(v, float) and math.isnan(v)):
            return "none"
        return f"{v:.1f}" if isinstance(v, float) else str(v)

    lines = ["| " + " | ".join(t.columns) + " |", "|" + "---|" * len(t.columns)]
    lines += ["| " + " | ".join(fmt(v) for v in r) + " |" for r in t.itertuples(index=False)]

    wl = ["", "Wheel (5010 + 6:1, hand calc, not simulated yet: needs e3 balance):",
          "| robot kg | max base accel m/s² | max lean it can catch |", "|---|---|---|"]
    for M in (10, 12, 14, 16, 18):
        acc = 2 * wheel.tau_peak / (rw * M)
        wl.append(f"| {M} | {acc:.1f} | {math.degrees(math.atan(acc / 9.81)):.0f}° |")
    return t, "\n".join(lines + wl)


def plot(df, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    series = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]      # categorical slots 1–4, fixed order by ratio
    ink, muted, grid = "#2b2b2a", "#6b6a64", "#e4e3dd"
    motors = list(dict.fromkeys(df.motor))
    ratios = sorted(df.ratio.unique())
    ncol = 2
    nrow = math.ceil(len(motors) / ncol)
    fig, axes = plt.subplots(nrow, ncol, figsize=(10, 3.8 * nrow), sharey=True, squeeze=False)
    flat = axes.ravel()
    for ax in flat[len(motors):]:
        ax.set_visible(False)
    for ax, motor in zip(flat, motors):
        for cm in JUMP_TARGETS_CM:
            ax.axhline(cm, color=muted, lw=0.8, ls="--", zorder=1)
            ax.text(0.99, cm + 0.5, f"target {cm} cm", color=muted, fontsize=7, ha="right",
                    transform=ax.get_yaxis_transform())
        for c, ratio in zip(series, ratios):
            sub = df[(df.motor == motor) & (df.ratio == ratio)].sort_values("mass_kg")
            ok = sub.hold_ok
            ax.plot(sub.mass_kg, sub.clearance_cm, color=c, lw=2, label=f"{ratio}:1", zorder=3)
            ax.plot(sub.mass_kg[ok], sub.clearance_cm[ok], "o", ms=5, color=c, mec="white", mew=1.5, zorder=4)
            ax.plot(sub.mass_kg[~ok], sub.clearance_cm[~ok], "x", ms=6, color=c, mew=1.5, zorder=4)
        ax.set_title(motor, color=ink, fontsize=10)
        ax.set_xlabel("robot mass, kg", color=muted, fontsize=8)
        ax.grid(color=grid, lw=0.6); ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(grid)
        ax.tick_params(colors=muted, labelsize=7, labelleft=True)
        ax.set_ylabel("wheel clearance, cm", color=muted, fontsize=8)
    flat[0].legend(title="gear ratio", fontsize=7, title_fontsize=7, frameon=False, loc="lower left")
    fig.suptitle("Jump height vs robot mass per leg motor   (● holds stand/crouch/lean within τ_cont, × overheats)",
                 color=ink, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=120, facecolor="white")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="only the mass_scale list in params.yaml")
    ap.add_argument("--workers", type=int, default=None)
    args = ap.parse_args()

    p = load_params()
    sw = p["sweep"]
    scales = sorted(set(sw["mass_scale"]) | (set() if args.quick else set(EXTRA_MASS_SCALES)))
    cases = [(p, m, r, s) for m in sw["leg_motor"] for r in sw["ratio"] for s in scales]
    print(f"{len(cases)} cases ({len(sw['leg_motor'])} motors × {len(sw['ratio'])} ratios × {len(scales)} masses)…")
    with Pool(args.workers) as pool:
        rows = pool.map(run_case, cases)
    df = judge(pd.DataFrame(rows))

    table, md = summary_table(df, p)
    out = ROOT / "results" / "sweep_motors" / datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy(PARAMS, out / "params.yaml")
    df.to_csv(out / "sweep.csv", index=False)
    plot(df, out / "sweep.png")
    text = ("# Motor sweep\n\nMax robot mass (kg) that passes. Every pass also requires 2-leg stand + crouch "
            f"({p['jump']['crouch_deg']}°) knee and 15° lean hip-roll holds ≤ τ_cont. 'none' = fails at the lightest mass tried "
            f"({df.mass_kg.min():.1f}–{df.groupby('motor').mass_kg.min().max():.1f} kg). "
            f"Masses tried go up to {df.mass_kg.max():.1f} kg; a value at the top of a combo's range means "
            "the limit is higher than tested.\n\n" + md + "\n")
    (out / "summary.md").write_text(text, encoding="utf-8")
    print(text)
    print(f"saved {out.relative_to(ROOT)}  (summary.md, sweep.csv, sweep.png, params.yaml)")


if __name__ == "__main__":
    main()
