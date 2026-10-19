"""Generate models/robot.xml from config/params.yaml.

Never hand-edit robot.xml: change params.yaml and re-run this.

  .\\run models\\build_mjcf.py

Kinematic tree (MuJoCo models are trees rooted at the torso; each joint lives
in the child body and connects it to its parent):

  torso (free joint = 6 DOF, floats in the world)
  ├── hip_cradle_L   hip_roll_L   (x axis, at the hip point)
  │   └── femur_L    hip_pitch_L  (y axis, same hip point)   + = thigh forward
  │       └── shin_L knee_L       (y axis)                   + = shin back (bent)
  │           └── wheel_L wheel_L (y axis)                   + = rolls forward
  ├── hip_cradle_R ... (mirror)
  └── neck           neck_pan     (z axis)
      └── head       head_tilt    (y axis)
          ├── eye_L  eye_L        (x axis, hinge at inner corner)  ┐ one motor on eye_L,
          └── eye_R  eye_R        (x axis)                         ┘ eye_R = -eye_L (equality)

Zero pose = legs straight down. Standing pose ("stand" keyframe), knee_dir: back
(knee points backward): hip_pitch = −θ, knee = −2θ, which keeps the wheel under the hip.
knee_dir: forward flips both signs.
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from actuators import load_actuators   # noqa: E402

PARAMS = ROOT / "config" / "params.yaml"
OUT = ROOT / "models" / "robot.xml"
G = 9.81
LEG = ("hip_roll", "hip_pitch", "knee")


def load_params():
    with open(PARAMS, encoding="utf-8") as f:
        return yaml.safe_load(f)


# +1 = knee points forward (human), −1 = knee points backward (bird/BDX). From robot.knee_dir.
KNEE_SIGN = -1 if load_params()["robot"].get("knee_dir", "back") == "back" else +1


def leg_pose(theta, roll=0.0):
    """Leg joint angles (rad) that keep the wheel under the hip, thigh at theta from vertical
    (theta is always positive; KNEE_SIGN picks which way the knee points).
    roll = hip roll angle (rad); + swings the wheel toward +y (left) on BOTH legs."""
    return {"hip_roll": roll, "hip_pitch": KNEE_SIGN * theta, "knee": KNEE_SIGN * 2 * theta}


def hip_roll_pose(lean=0.0, spread=0.0):
    """Hip roll angles {side: rad} for the two things roll is used for:
      lean   + = body leans LEFT over the wheels (wheels swing right), e.g. carving a left turn.
               Both legs tilt the same way; body stays level and shifts sideways (parallelogram).
      spread + = wide stance (wheels swing outward), − = narrow.
    The joint axis is +x on both legs, so + roll moves either wheel toward +y."""
    return {"L": -lean + spread, "R": -lean - spread}


# ---------- inertia helpers ----------
# Each part is (mass, position in body frame, 3x3 inertia about its own centre).

def box_I(m, size):
    """Solid box, size = full side lengths [x, y, z]."""
    x, y, z = size
    return np.diag([m * (y * y + z * z), m * (x * x + z * z), m * (x * x + y * y)]) / 12


def cyl_y_I(m, r, w):
    """Solid cylinder with its axis along y (wheels, pancake motors)."""
    side = m * (3 * r * r + w * w) / 12
    return np.diag([side, m * r * r / 2, side])


def cyl_x_I(m, r, w):
    """Same, axis along x (hip-roll actuators)."""
    side = m * (3 * r * r + w * w) / 12
    return np.diag([m * r * r / 2, side, side])


def combine(parts):
    """Lump several parts into one body: total mass, centre of mass, inertia about the CoM."""
    M = sum(m for m, _, _ in parts)
    com = sum(m * np.asarray(p, float) for m, p, _ in parts) / M
    I = np.zeros((3, 3))
    for m, p, Ii in parts:
        d = np.asarray(p, float) - com
        I += Ii + m * (d @ d * np.eye(3) - np.outer(d, d))   # parallel-axis theorem
    return M, com, I


def inertial(parts):
    M, com, I = combine(parts)
    return (f'<inertial pos="{vec(com)}" mass="{M:.4f}" '
            f'fullinertia="{I[0,0]:.6g} {I[1,1]:.6g} {I[2,2]:.6g} {I[0,1]:.6g} {I[0,2]:.6g} {I[1,2]:.6g}"/>')


def vec(v):
    return " ".join(f"{x:.4f}" for x in v)


def joint_xml(name, axis, act, cfg, rng=None, extra=""):
    lim = f'limited="true" range="{rng[0]} {rng[1]}"' if rng else 'limited="false"'
    return (f'<joint name="{name}" type="hinge" axis="{axis}" {lim} armature="{act.armature:.5g}" '
            f'damping="{cfg["damping"]}" frictionloss="{cfg["frictionloss"]}" {extra}/>')


# ---------- model ----------

def build(p, base="free"):
    """base: 'free'     = floating body (the real robot)
             'fixed'    = body bolted in the air
             'vertical' = test rig: body can only slide up/down (no tipping), for jump tests
             'roll'     = test rig: body slides up/down + sideways and can roll, but can't pitch,
                          for hip-roll tests without a balance controller"""
    r = p["robot"]
    k = r["mass_scale"]
    m = {n: v * k for n, v in r["masses"].items()}
    L1, L2 = r["link_femur"], r["link_shin"]
    rw, ww = r["wheel_radius"], r["wheel_width"]
    bx, by, bz = r["body_size"]
    hx, hy, hz = r["hip_x"], r["hip_y"], r["hip_z"]
    ly, wy = r["leg_plane_y"], r["wheel_offset_y"]
    rng = r["joint_range_deg"]
    acts = load_actuators(p)
    cfg = p["actuators"]
    th = math.radians(r["stand_theta_deg"])

    # Leg actuator mass follows the motor choice (motor + gearbox/housing), so a heavy motor costs weight.
    gearbox = p.get("sweep", {}).get("gearbox_mass")
    act_mass = {j: k * (acts[j].mass + gearbox) if (acts[j].mass and gearbox is not None) else m["leg_actuator"]
                for j in LEG}

    # Torso: shell + battery + electronics + misc + both hip-roll actuators (they live in the body, behind the hip)
    torso_parts = [
        (m["body_shell"], [0, 0, 0], box_I(m["body_shell"], r["body_size"])),
        (m["battery"], [0, 0, r["battery_z"]], box_I(m["battery"], [0.16, 0.05, 0.06])),
        (m["electronics"], [0, 0, 0.05], box_I(m["electronics"], [0.15, 0.20, 0.05])),
        (m["misc"], [0, 0, 0], box_I(m["misc"], r["body_size"])),
    ]
    for s in (+1, -1):
        torso_parts.append((act_mass["hip_roll"], [hx - 0.08, s * hy, hz], cyl_x_I(act_mass["hip_roll"], 0.056, 0.05)))

    # Head stage: neck pan part, head block, two eye pods
    m_neck = m["head"] * r["neck_frac"]
    m_eye = r["eye_mass"] * k
    m_head = m["head"] - m_neck - 2 * m_eye
    px, py, pz = r["eye_pod"]
    sign_pod = +1 if r["eye_hinge"] == "inner" else -1           # pod extends outward from an inner hinge
    hinge_y = r["eye_spacing"] / 2 - sign_pod * py / 2

    # Knee spring: torque at stand = spring_frac × static knee torque (two legs, whole mass)
    M_total = (sum(pm for pm, _, _ in torso_parts) + m["head"]
               + 2 * (act_mass["hip_pitch"] + act_mass["knee"] + m["hip_cradle"] + m["femur"] + m["shin"]
                      + m["wheel_actuator"] + m["wheel"]))
    tau_knee_stand = (M_total * G / 2) * L1 * math.sin(th)
    k_spring = r["spring_frac"] * tau_knee_stand / (2 * th)   # springref = 0 (straight leg)

    def leg(side, s):
        """side = 'L' or 'R', s = +1 (left, +y) or -1."""
        dw = wy - ly                                            # leg plane → wheel centre, outward
        cradle = inertial([
            (m["hip_cradle"], [0, 0, 0], box_I(m["hip_cradle"], [0.10, 0.08, 0.10])),
            # hip-pitch actuator outboard, knee actuator inboard, coaxial on the pitch axis
            (act_mass["hip_pitch"], [0, s * 0.03, 0], cyl_y_I(act_mass["hip_pitch"], 0.056, 0.05)),
            (act_mass["knee"], [0, -s * 0.03, 0], cyl_y_I(act_mass["knee"], 0.056, 0.05)),
        ])
        femur = inertial([(m["femur"], [0, 0, -L1 / 2], box_I(m["femur"], [0.04, 0.03, L1]))])
        shin = inertial([
            (m["shin"], [0, 0, -L2 / 2], box_I(m["shin"], [0.04, 0.03, L2])),
            (m["wheel_actuator"], [0, s * dw / 2, -L2], cyl_y_I(m["wheel_actuator"], 0.03, 0.05)),  # 5010 stator at hub
        ])
        wheel = inertial([(m["wheel"], [0, 0, 0], cyl_y_I(m["wheel"], rw, ww))])
        return f"""
    <body name="hip_cradle_{side}" pos="{hx} {s * hy} {hz}">
      {joint_xml(f"hip_roll_{side}", "1 0 0", acts["hip_roll"], cfg["hip_roll"], rng["hip_roll"])}
      {cradle}
      <geom class="vis" type="box" size="0.05 0.04 0.05" rgba="0.35 0.35 0.4 1"/>
      <geom class="vis" type="cylinder" size="0.056 0.025" pos="0 {s * 0.03:.4f} 0" euler="90 0 0" rgba="0.2 0.2 0.25 1"/>
      <geom class="vis" type="cylinder" size="0.056 0.025" pos="0 {-s * 0.03:.4f} 0" euler="90 0 0" rgba="0.25 0.25 0.3 1"/>
      <geom class="vis" type="sphere" size="0.015" rgba="1 1 1 1"/>
      <body name="femur_{side}" pos="0 {s * ly:.4f} 0">
        {joint_xml(f"hip_pitch_{side}", "0 -1 0", acts["hip_pitch"], cfg["hip_pitch"], rng["hip_pitch"])}
        {femur}
        <geom class="body" type="capsule" fromto="0 0 0 0 0 {-L1}" size="0.02" rgba="0.85 0.85 0.9 1"/>
        <body name="shin_{side}" pos="0 0 {-L1}">
          {joint_xml(f"knee_{side}", "0 1 0", acts["knee"], cfg["knee"], rng["knee"],
                     f'stiffness="{k_spring:.4f}" springref="0"')}
          {shin}
          <geom class="body" type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.018" rgba="0.85 0.85 0.9 1"/>
          <geom class="vis" type="capsule" fromto="0 0 {-L2} 0 {s * dw:.4f} {-L2}" size="0.012" rgba="0.3 0.3 0.3 1"/>
          <body name="wheel_{side}" pos="0 {s * dw:.4f} {-L2}">
            {joint_xml(f"wheel_{side}", "0 1 0", acts["wheel"], cfg["wheel"])}
            {wheel}
            <geom name="wheel_{side}" class="wheel" type="cylinder" size="{rw} {ww / 2}" euler="90 0 0"/>
            <geom class="vis" type="box" size="{rw * 0.9:.3f} {ww / 2 + 0.002:.3f} 0.008" rgba="0.9 0.6 0.1 1"/>
          </body>
        </body>
      </body>
    </body>"""

    def eye(side, s):
        pod_c = [0, s * sign_pod * py / 2, 0]                   # pod centre rel. its hinge
        return f"""
          <body name="eye_{side}" pos="{r["eye_face_x"] + px / 2:.4f} {s * hinge_y:.4f} {r["eye_z"]}">
            {joint_xml(f"eye_{side}", "1 0 0", acts["eye_tilt"], cfg["eye_tilt"], rng["eye_tilt"])}
            {inertial([(m_eye, pod_c, box_I(m_eye, r["eye_pod"]))])}
            <geom class="vis" type="box" size="{px / 2} {py / 2} {pz / 2}" pos="{vec(pod_c)}" rgba="0.15 0.15 0.18 1"/>
            <geom class="vis" type="cylinder" size="{r["eye_lens_radius"]} 0.004" pos="{px / 2:.4f} {pod_c[1]:.4f} 0" euler="0 90 0" rgba="0.1 0.5 0.9 1"/>
            <geom class="vis" type="sphere" size="0.006" rgba="1 1 1 1"/>
          </body>"""

    hsx, hsy, hsz = r["head_size"]
    hc = r["head_centre"]
    nl = r["neck_length"]

    z_hip = rw + (L1 + L2) * math.cos(th)
    z_torso = z_hip - hz                                         # wheel just touching the floor at stand
    if base == "fixed":
        z_torso += 0.05                                          # pinned: legs hang 5 cm clear so roll/pitch aren't blocked by the floor
    # Keyframe qpos follows the tree order: [root] L(roll pitch knee wheel) R(...) neck head eye_L eye_R
    base_joint, root_q = {
        "free": ('<freejoint name="root"/>', f"0 0 {z_torso:.4f} 1 0 0 0 "),
        "fixed": ("", ""),
        "vertical": ('<joint name="rig_z" type="slide" axis="0 0 1"/>', "0 "),  # slide qpos = offset from start
        "roll": ('<joint name="rig_z" type="slide" axis="0 0 1"/>'
                 '<joint name="rig_y" type="slide" axis="0 1 0"/>'
                 '<joint name="rig_roll" type="hinge" axis="1 0 0"/>', "0 0 0 "),
    }[base]
    lp = leg_pose(th)
    leg_q = f"{lp['hip_roll']} {lp['hip_pitch']:.5f} {lp['knee']:.5f} 0"

    act_xml = []
    for side in "LR":
        for j in LEG + ("wheel",):
            t = acts[j].tau_peak
            act_xml.append(f'<motor name="{j}_{side}" joint="{j}_{side}" gear="1" ctrllimited="true" ctrlrange="{-t:.3f} {t:.3f}"/>')
    for name, joint in (("neck_pan", "neck_pan"), ("head_tilt", "head_tilt"), ("eye_tilt", "eye_L")):
        t = acts[name].tau_peak
        act_xml.append(f'<motor name="{name}" joint="{joint}" gear="1" ctrllimited="true" ctrlrange="{-t:.4f} {t:.4f}"/>')

    equality = ""
    if cfg["eye_tilt"].get("coupling") == "mirror":
        # joint1 = polycoef(joint2): eye_R = -1 × eye_L → both outer corners move up together
        equality = '<equality><joint name="eye_mirror" joint1="eye_R" joint2="eye_L" polycoef="0 -1 0 0 0"/></equality>'

    joints = [f"{j}_{s}" for s in "LR" for j in LEG + ("wheel",)] + ["neck_pan", "head_tilt", "eye_L", "eye_R"]
    actuators = [f"{j}_{s}" for s in "LR" for j in LEG + ("wheel",)] + ["neck_pan", "head_tilt", "eye_tilt"]
    sensors = [f'<jointpos name="q_{j}" joint="{j}"/><jointvel name="qd_{j}" joint="{j}"/>' for j in joints]
    sensors += [f'<actuatorfrc name="tau_{a}" actuator="{a}"/>' for a in actuators]

    cf = p["contact"]["wheel_friction"]
    return f"""<mujoco model="the7_v2">
  <!-- GENERATED by models/build_mjcf.py from config/params.yaml. Do not edit by hand. -->
  <compiler angle="degree" inertiafromgeom="false"/>
  <option timestep="{p["sim"]["timestep"]}" gravity="0 0 {-G}"/>

  <default>
    <!-- Collision filter: robot parts (contype 2) only collide with the floor (conaffinity 2),
         never with each other. -->
    <default class="body"><geom contype="2" conaffinity="1"/></default>
    <default class="vis"><geom contype="0" conaffinity="0"/></default>
    <default class="wheel"><geom contype="2" conaffinity="1" condim="3" friction="{cf[0]} {cf[1]} {cf[2]}" rgba="0.15 0.15 0.15 1"/></default>
  </default>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="5 5 0.1" contype="1" conaffinity="2" rgba="0.75 0.75 0.75 1"/>

    <body name="torso" pos="0 0 {z_torso:.4f}">
      {base_joint}
      {inertial(torso_parts)}
      <geom class="body" type="box" size="{bx / 2} {by / 2} {bz / 2}" rgba="0.55 0.35 0.85 0.6"/>
      <site name="imu" pos="0 0 0"/>
{leg("L", +1)}
{leg("R", -1)}
      <body name="neck" pos="{vec(r["neck_base"])}">
        {joint_xml("neck_pan", "0 0 1", acts["neck_pan"], cfg["neck_pan"], rng["neck_pan"])}
        {inertial([(m_neck, [0, 0, nl / 2], box_I(m_neck, [0.08, 0.08, nl]))])}
        <geom class="vis" type="cylinder" size="0.035 {nl / 2}" pos="0 0 {nl / 2}" rgba="0.3 0.3 0.35 1"/>
        <body name="head" pos="0 0 {nl}">
          {joint_xml("head_tilt", "0 1 0", acts["head_tilt"], cfg["head_tilt"], rng["head_tilt"])}
          {inertial([(m_head, hc, box_I(m_head, r["head_size"]))])}
          <geom class="body" type="box" size="{hsx / 2} {hsy / 2} {hsz / 2}" pos="{vec(hc)}" rgba="0.9 0.9 0.95 1"/>
          <site name="eyes" pos="{r["eye_face_x"]} 0 {r["eye_z"]}"/>
{eye("L", +1)}
{eye("R", -1)}
        </body>
      </body>
    </body>
  </worldbody>

  {equality}

  <actuator>
    {chr(10).join("    " + a for a in act_xml).strip()}
  </actuator>

  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    {chr(10).join("    " + s for s in sensors).strip()}
  </sensor>

  <keyframe>
    <key name="stand" qpos="{root_q}{leg_q} {leg_q} 0 0 0 0"/>
  </keyframe>
</mujoco>
"""


def report(model, p):
    import mujoco
    r = p["robot"]
    print(f"{'body':<14}{'mass kg':>9}")
    for i in range(1, model.nbody):
        print(f"{model.body(i).name:<14}{model.body_mass[i]:9.3f}")
    print(f"{'TOTAL':<14}{mujoco.mj_getTotalmass(model):9.3f}   (mass_scale {r['mass_scale']})")
    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, 0)
    mujoco.mj_forward(model, d)
    print(f"\nstand θ={r['stand_theta_deg']}°: hip axis z = {d.body('femur_L').xpos[2]:.3f} m, "
          f"eyes z = {d.site('eyes').xpos[2]:.3f} m, "
          f"track = {d.body('wheel_L').xpos[1] - d.body('wheel_R').xpos[1]:.3f} m")
    ahead = (d.subtree_com[1][0] - d.body("wheel_L").xpos[0]) * 1000
    print(f"robot CoM: {ahead:+.1f} mm ahead of the wheel axles (+ = tips forward), z = {d.subtree_com[1][2]:.3f} m")
    print(f"knee spring k = {model.jnt_stiffness[model.joint('knee_L').id]:.3f} N·m/rad\n")

    acts = load_actuators(p)
    print(f"{'actuator':<10}{'motor':<11}{'N':>4}{'τ_peak':>8}{'τ_cont':>8}{'ω0':>7}{'armature':>10}")
    for j, a in acts.items():
        c = p["actuators"][j]
        print(f"{j:<10}{c['motor']:<11}{c['ratio']:>4}{a.tau_peak:8.2f}{a.tau_cont:8.2f}{a.w0:7.1f}{a.armature:10.5f}")
    print("(N·m, N·m, rad/s at the joint, kg·m²)")


if __name__ == "__main__":
    import mujoco
    p = load_params()
    xml = build(p)
    OUT.write_text(xml, encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(OUT))   # compile check
    print(f"wrote {OUT.relative_to(ROOT)}  (nq={model.nq}, nv={model.nv}, nu={model.nu})\n")
    report(model, p)
