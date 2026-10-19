"""Step 1: MuJoCo basics.

Two things in one scene:
  * a 1 kg box with a free joint (6 DOF) dropped onto a floor
  * a femur-sized arm on a fixed pivot (hinge, 1 DOF) held at 45 deg by a PD
    controller driving a motor actuator

Run:
  python tutorials/01_basics.py              # opens the viewer
  python tutorials/01_basics.py --headless   # just prints the numbers

Things to try (edit and re-run):
  * set armature="0" on the shoulder joint and watch the arm overshoot more
  * drop ctrlrange to "-0.4 0.4" so the motor can't hold the arm up (gravity needs ~0.48 N·m at 45 deg)
  * raise KP and watch the steady-state error (45 - arm deg) shrink: error ≈ gravity torque / KP
  * change the box mass or start height
"""
import argparse
import math
import time

import mujoco
import mujoco.viewer

# MJCF = MuJoCo's XML model format. Later this string gets generated from params.yaml.
XML = """
<mujoco model="step1_basics">
  <!-- Physics step: 1 ms. Contact gets unstable if this is much bigger. -->
  <option timestep="0.001" gravity="0 0 -9.81"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.8 0.8 0.8 1"/>

    <!-- Falling box. freejoint = 6 DOF: qpos gets 7 numbers (x y z + quaternion w x y z),
         qvel gets 6 (linear + angular velocity). -->
    <body name="box" pos="0 0 1">
      <freejoint name="box_free"/>
      <!-- Explicit inertial: mass + diagonal inertia of a 0.2 m cube, I = m*s^2/6 -->
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.00667 0.00667 0.00667"/>
      <geom type="box" size="0.1 0.1 0.1" rgba="0.6 0.3 0.9 1"/>
    </body>

    <!-- Arm on a fixed pivot. hinge = 1 DOF rotating about y (side-to-side, like hip pitch).
         armature = motor rotor inertia felt at the joint = J_rotor * N^2 = 1.2e-4 * 11^2 = 0.0145 -->
    <body name="arm" pos="0.6 0 0.5">
      <joint name="shoulder" type="hinge" axis="0 1 0" damping="0.05" armature="0.0145"/>
      <inertial pos="0 0 -0.14" mass="0.5" diaginertia="0.0033 0.0033 0.0001"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.28" size="0.02" rgba="0.2 0.6 0.9 1"/>
    </body>
  </worldbody>

  <!-- A motor is a pure torque source: torque = gear * ctrl. ctrlrange = 8108 + 11:1 peak. -->
  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="1" ctrllimited="true" ctrlrange="-19.6 19.6"/>
  </actuator>

  <sensor>
    <jointpos name="shoulder_pos" joint="shoulder"/>
    <actuatorfrc name="shoulder_frc" actuator="shoulder_motor"/>
  </sensor>
</mujoco>
"""

TARGET = math.radians(45.0)
KP, KD = 20.0, 1.0          # N·m/rad, N·m·s/rad
SIM_SECONDS = 2.0
PRINT_EVERY = 0.25           # s


def control(model, data):
    """PD: torque = kp*(target - angle) - kd*velocity."""
    q = data.qpos[model.jnt_qposadr[model.joint("shoulder").id]]
    qd = data.qvel[model.jnt_dofadr[model.joint("shoulder").id]]
    data.ctrl[0] = KP * (TARGET - q) - KD * qd


def report(model, data, header=False):
    if header:
        print(f"{'t':>5} | {'box z':>6} {'box vz':>7} | {'arm deg':>7} {'ctrl':>6} "
              f"{'act_force':>9} {'qfrc_act':>8} {'sensor':>7}")
    sh = model.joint("shoulder")
    box_z = data.qpos[2]                       # free joint: qpos[0:3] = position
    box_vz = data.qvel[2]                      # qvel[0:3] = linear velocity
    arm = data.qpos[sh.qposadr[0]]
    print(f"{data.time:5.2f} | {box_z:6.3f} {box_vz:7.3f} | {math.degrees(arm):7.2f} "
          f"{data.ctrl[0]:6.3f} {data.actuator_force[0]:9.3f} "
          f"{data.qfrc_actuator[sh.dofadr[0]]:8.3f} {data.sensor('shoulder_frc').data[0]:7.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true", help="no viewer, just print")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    print(f"nq (qpos size) = {model.nq}, nv (qvel size) = {model.nv}, nu (actuators) = {model.nu}")
    print(f"total mass = {mujoco.mj_getTotalmass(model):.2f} kg\n")

    next_print = 0.0
    report(model, data, header=True)

    if args.headless:
        while data.time < SIM_SECONDS:
            control(model, data)
            mujoco.mj_step(model, data)
            if data.time >= next_print:
                report(model, data)
                next_print += PRINT_EVERY
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            t0 = time.perf_counter()
            control(model, data)
            mujoco.mj_step(model, data)
            viewer.sync()
            if data.time >= next_print and data.time <= SIM_SECONDS:
                report(model, data)
                next_print += PRINT_EVERY
            # run at real time
            dt = model.opt.timestep - (time.perf_counter() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
