# The 7 / V2 sizing sim: every tunable lives here. Scripts read this file;
# a sweep should never require editing code.
# Units: SI everywhere (m, kg, s, N·m, rad unless the key says _deg).
# Tags: VERIFIED (datasheet/measured) · DERIVED (calculated from verified data) · ESTIMATE (guess, replace later)
#
# Frames: BODY frame origin = centre of the body box, x forward, y left, z up.
# Numbers below match the 3D body mockup (hip cradles in the lower body, belt knee, Ø160 wheels).

robot:
  mass_scale: 1.0             # multiply all masses; sweep 0.85–1.35 for 10–16 kg
  link_femur: 0.280
  link_shin: 0.280
  wheel_radius: 0.080
  wheel_width: 0.040
  body_size: [0.270, 0.380, 0.310]   # x (front–back), y (side), z
  stand_theta_deg: 30         # thigh angle from vertical at stand (hip height = 2·L·cos θ + wheel_radius)
  spring_frac: 0.30           # fraction of body weight carried by knee springs at stand

  # ── HIP ──────────────────────────────────────────────────────────────
  # Each hip is one point where two axes cross (the white sphere in the mockup):
  #   • hip ROLL axis: points front–back (x). Tilts the whole leg sideways.
  #   • hip PITCH axis: points side–side (y). Swings the thigh forward/back.
  #   • KNEE axis: parallel to pitch, one femur length further down the leg.
  #   • WHEEL axis: parallel to pitch, at the end of the shin.
  # Chain (parent → child): body → hip_roll → hip_pitch → knee → wheel.
  # These 3 numbers place that crossing point in the body frame (left leg; right leg mirrors y).
  hip_x: 0.045                # hip is 45 mm forward of body centre (roll actuator sits behind it). DERIVED from mockup
  hip_y: 0.099                # lateral offset of the hip point (centre of the pitch+knee stack). DERIVED from mockup
  hip_z: -0.070               # hip point is 70 mm below body centre (legs exit the belly). DERIVED from mockup
  leg_plane_y: 0.009          # femur/shin sit 9 mm outboard of the hip point. ESTIMATE
  wheel_offset_y: 0.062       # wheel centre is 62 mm outboard of the hip point → track ≈ 2·(0.099+0.062) = 0.322 m. DERIVED
  # (old `track` and `hip_roll_y` keys removed: track is now computed from hip_y + wheel_offset_y)

  # ── HEAD ─────────────────────────────────────────────────────────────
  # Chain: body → neck_pan (z axis) → head_tilt (y axis) → head → eye_L / eye_R (x axes, linked)
  neck_base: [0.065, 0.000, 0.155]   # neck-pan axis where it meets the body top (front half). DERIVED from mockup
  neck_length: 0.070          # body top → head-tilt axis. DERIVED from mockup
  head_size: [0.180, 0.230, 0.130]   # x, y, z of the head block. DERIVED from mockup
  head_centre: [0.020, 0.000, 0.075] # head block centre rel. head-tilt axis. DERIVED
  neck_frac: 0.35             # fraction of `head` mass that is the neck pan stage (stays on the body side of tilt). ESTIMATE

  # ── EYES: two segmented eye pods, linked like ornithopter wings ──────
  # Each eye is its own pod on the head's front face. Each pod hinges about an
  # axis pointing FORWARD (x) at its INNER corner (next to the nose), so the OUTER
  # corner swings up/down in an arc. The two pods are mirrored: when the left
  # outer corner goes up, the right outer corner goes up too (like wings flapping).
  # → happy/sad/angry brows without moving the head. One motor drives both through a linkage;
  #   in MuJoCo that's ONE actuator on eye_L plus an equality constraint eye_R = -eye_L.
  eye_spacing: 0.110          # eye centre to eye centre. DERIVED from mockup (±55 mm)
  eye_pod: [0.020, 0.075, 0.045]     # pod block x (depth), y (width), z (height). ESTIMATE, visual + inertia
  eye_face_x: 0.110           # front face of head rel. head-tilt axis (pods sit here). DERIVED
  eye_z: 0.080                # eye height rel. head-tilt axis → eyes ≈ 0.94 m off the floor at stand. DERIVED
  eye_hinge: inner            # hinge at the inner corner (y = ±(eye_spacing/2 − eye_pod_y/2) = ±17.5 mm)
  eye_lens_radius: 0.024      # camera lens, visual only
  eye_mass: 0.035             # per pod: camera module + bezel + shell. ESTIMATE

  battery_z: -0.110           # battery centre height in body (low + centred = stable). DERIVED from mockup

  joint_range_deg:            # soft stops for the sim
    hip_roll: [-15, 15]       # limited by cradle-to-body clearance in the mockup
    hip_pitch: [-60, 120]     # + = thigh swings forward
    knee: [0, 150]            # 0 = straight leg, + = knee bends (shin swings back)
    neck_pan: [-90, 90]       # look left/right
    head_tilt: [-30, 30]      # nod (was neck_UD)
    eye_tilt: [-25, 25]       # outer-corner arc; + = outer corners UP on both eyes (mirrored)

  masses:                     # kg, all ESTIMATE until weighed unless tagged
    leg_actuator: 0.60        # 8108 = 0.271 VERIFIED + ~0.33 gearbox/bearings/housing ESTIMATE
    wheel_actuator: 0.33      # 5010 = 0.170 VERIFIED + 6:1 planetary + housing
    wheel: 0.30
    femur: 0.25               # incl. belt + pulleys
    shin: 0.25                # incl. knee spring
    hip_cradle: 0.30
    body_shell: 1.50
    battery: 0.70             # 6S ~5 Ah
    electronics: 0.80
    head: 1.20                # total head stage incl. neck pan + tilt servo + eyes
    misc: 0.80

electrical:
  bus_voltage: 22.2           # 6S nominal; sweep 21–25
  servo_bus_voltage: 12.0     # STS3215 bus (step-down from 6S)

motors:
  m5010_340:  {kv: 340, R: 0.27,  kt: 0.027,  i_peak: 20, i_cont: 6,  tau_peak: 0.54, mass: 0.170, J_rotor: 3.0e-5}  # R VERIFIED, kt DERIVED from prop test; J ESTIMATE
  m8108_100:  {kv: 100, R: 0.163, kt: 0.100,  i_peak: 34, i_cont: 11, tau_peak: 3.3,  mass: 0.271, J_rotor: 1.8e-4}  # R, 34.3 A, 271 g VERIFIED; kt DERIVED from dyno; i_cont + J ESTIMATE
  m6215_170:  {kv: 170, R: 0.071, kt: 0.054,  i_peak: 39, i_cont: 13, tau_peak: 2.08, mass: 0.425, J_rotor: 1.6e-4}  # 2.08 N·m + 425 g VERIFIED; R/current DERIVED from 320KV
  m10_100:    {kv: 100, R: 0.080, kt: 0.0955, i_peak: 63, i_cont: 20, tau_peak: 6.0,  mass: 0.597, J_rotor: 6.8e-4}  # MAD M10 IPE, big-motor reference
  # Geared servos / gimbal motors: modelled as torque sources with their own limits (no kt/ratio math).
  sts3215:    {type: servo, tau_peak: 2.94, tau_cont: 1.0,  speed: 4.76, mass: 0.055}   # Feetech, 30 kg·cm @ 12 V, 0.22 s/60° VERIFIED; tau_cont ESTIMATE (≈⅓ stall)
  sts3032:    {type: servo, tau_peak: 0.44, tau_cont: 0.15, speed: 11.6, mass: 0.020}   # Feetech micro, 4.5 kg·cm stall / 1.5 rated @ 6 V VERIFIED (alt. eye motor)
  gm2804:     {type: gimbal, tau_peak: 0.034, tau_cont: 0.025, speed: 15.0, mass: 0.051, R: 9.0}  # iPower, 250–350 g·cm VERIFIED; speed ESTIMATE (limited by FOC tuning)

actuators:
  # frictionloss = breakaway torque at the OUTPUT [N·m]; damping = viscous at the output [N·m·s/rad].
  # Back-EMF is already in the torque–speed clamp → don't add it again as damping. All friction/damping ESTIMATE.
  hip_roll:  {motor: m8108_100, ratio: 11, eta: 0.85, damping: 0.10, frictionloss: 0.30}
  hip_pitch: {motor: m8108_100, ratio: 11, eta: 0.85, damping: 0.10, frictionloss: 0.30}
  knee:      {motor: m8108_100, ratio: 11, eta: 0.83, damping: 0.10, frictionloss: 0.30}  # extra ~2% for the belt
  wheel:     {motor: m5010_340, ratio: 6,  eta: 0.90, damping: 0.01, frictionloss: 0.03}
  neck_pan:  {motor: m5010_340, ratio: 6,  eta: 0.90, damping: 0.02, frictionloss: 0.03}
  head_tilt: {motor: sts3215,   ratio: 1,  eta: 1.0,  damping: 0.05, frictionloss: 0.05}
  eye_tilt:  {motor: gm2804,    ratio: 1,  eta: 0.90, damping: 0.001, frictionloss: 0.003, coupling: mirror}  # ONE motor drives both pods; η 0.9 for the linkage

contact:
  wheel_friction: [1.0, 0.005, 0.0001]   # sliding, torsional, rolling. ESTIMATE
  # Braking / standing still: no extra friction needed. The wheel motors hold position under FOC
  # (that IS the brake). In sim: switch the wheel from velocity control to a stiff position PD.
  floor: plane

sim:
  timestep: 0.001
  control_hz: 500

jump:                         # sim/experiments/e4_jump.py
  base: vertical              # vertical = test rig (slides up/down, can't tip); free = no rig (will likely tip)
  crouch_deg: 60              # θ at the bottom of the crouch (stand is stand_theta_deg)
  extend_deg: 5               # push-off target θ (near-straight leg)
  tuck_deg: 60                # θ in flight: pulls the wheels up for clearance
  land_deg: 30                # θ the legs aim for on landing
  t_settle: 0.5               # s, hold stand before starting
  t_crouch: 1.0               # s, time to sink into the crouch
  t_hold: 0.3                 # s, pause at the bottom
  t_total: 3.5                # s, whole experiment
  gains:                      # PD gains per phase [kp N·m/rad, kd N·m·s/rad]
    pose: [100, 3]            # stand / crouch / tuck
    push: [800, 4]            # knee during push: high kp = ask for more than the motor has → max effort
    push_hip: [300, 6]        # hip during push: steers (keeps wheel under hip), doesn't lift
    land: [60, 4]             # soft = compliant landing