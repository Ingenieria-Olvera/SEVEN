"""Look at the robot in the viewer, holding the stand pose with PD control.

  .\\run models\\view_robot.py          # body pinned in the air (can't fall), legs hold pose
  .\\run models\\view_robot.py --free   # body free: it WILL tip over (no balance controller yet)
  .\\run models\\view_robot.py --theta 50    # try a different stand angle (crouch)
  .\\run models\\view_robot.py --eyes 20     # eye pods' outer corners up 20° (mirrored linkage)
  .\\run models\\view_robot.py --hip_roll 10 # both hip rolls +10° (wheels swing toward +y / left)
  .\\run models\\view_robot.py --lean 15     # lean left 15° (carve pose: body shifts over the wheels)
  .\\run models\\view_robot.py --spread -10  # narrow stance (+ = wide)

The model is rebuilt from config/params.yaml every run, so edit the YAML and re-run.
"""
import argparse
import math
import time

import mujoco
import mujoco.viewer

from build_mjcf import build, hip_roll_pose, leg_pose, load_params

# {actuator name: (kp, kd)}. Target = the joint's keyframe value (eyes: --eyes).
GAINS = {**{f"{j}_{s}": (100.0, 3.0) for s in "LR" for j in ("hip_roll", "hip_pitch", "knee")},
         "neck_pan": (20.0, 0.5), "head_tilt": (20.0, 0.5), "eye_tilt": (0.5, 0.01)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--free", action="store_true", help="unpin the body")
    ap.add_argument("--theta", type=float, help="stand angle in degrees (overrides params.yaml)")
    ap.add_argument("--eyes", type=float, default=0.0, help="eye tilt target in degrees")
    ap.add_argument("--hip_roll", type=float, default=0.0, help="raw hip roll on both legs, degrees")
    ap.add_argument("--lean", type=float, default=0.0, help="lean left (+) / right (-) with hip roll, degrees")
    ap.add_argument("--spread", type=float, default=0.0, help="wide (+) / narrow (-) stance with hip roll, degrees")
    args = ap.parse_args()

    p = load_params()
    if args.theta is not None:
        p["robot"]["stand_theta_deg"] = args.theta
    model = mujoco.MjModel.from_xml_string(build(p, base="free" if args.free else "fixed"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)

    # Leg targets come from the pose functions in build_mjcf.py; head holds 0, eyes follow --eyes.
    rad = math.radians
    roll = hip_roll_pose(rad(args.lean), rad(args.spread))
    targets = {"eye_tilt": rad(args.eyes)}
    for s in "LR":
        pose = leg_pose(rad(p["robot"]["stand_theta_deg"]), roll[s] + rad(args.hip_roll))
        targets.update({f"{j}_{s}": q for j, q in pose.items()})

    # Look up each actuator's joint slots once.
    idx = []
    for a, (kp, kd) in GAINS.items():
        ai = model.actuator(a).id
        jid = model.actuator_trnid[ai, 0]
        qa, va = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
        idx.append((qa, va, ai, targets.get(a, 0.0), kp, kd))

    def control():
        for qa, va, ai, target, kp, kd in idx:
            data.ctrl[ai] = kp * (target - data.qpos[qa]) - kd * data.qvel[va]

    next_print = 0.5
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            t0 = time.perf_counter()
            control()
            mujoco.mj_step(model, data)
            viewer.sync()
            if data.time >= next_print:
                tau = {j: data.actuator(j).force[0] for j in ("hip_pitch_L", "knee_L", "hip_roll_L")}
                pitch = math.degrees(2 * math.asin(data.qpos[5])) if args.free else 0.0  # quat y ≈ body pitch
                eyes = [math.degrees(data.joint(e).qpos[0]) for e in ("eye_L", "eye_R")]
                print(f"t={data.time:5.2f}  hip_pitch_L={tau['hip_pitch_L']:6.2f}  knee_L={tau['knee_L']:6.2f}  "
                      f"hip_roll_L={tau['hip_roll_L']:6.2f} N·m  eyes L/R={eyes[0]:5.1f}/{eyes[1]:5.1f}°"
                      + (f"  body pitch={pitch:6.1f}°" if args.free else ""))
                next_print += 0.5
            dt = model.opt.timestep - (time.perf_counter() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
