# The 7 / V2 sizing sim: every tunable lives here. Scripts read this file;
# a sweep should never require editing code.
# Units: SI everywhere (m, kg, s, N·m, rad). Anything marked ESTIMATE is a guess
# to be replaced with measured/datasheet values.

# THIS WAS V1, V2 is the one being used. If not than updated is previous .md version +1
robot:
  mass_scale: 1.0             # multiply all masses; sweep 0.85–1.35 for 10–16 kg
  link_femur: 0.280
  link_shin: 0.280
  wheel_radius: 0.080
  wheel_width: 0.040
  track: 0.300
  body_size: [0.270, 0.380, 0.310]   
  # x (front–back), y (side), z
  stand_theta_deg: 30         # thigh angle from vertical at stand (hip height = 2·L·cos θ)
  spring_frac: 0.30           # fraction of body weight carried by knee springs at stand
  # Geometry of where things attach (body frame: origin at body-box centre,
  # x forward, y left, z up). Sketch these from CAD to replace the guesses.

  #! HIP AXSIS
  # Unsure exactly what this part means or maybe just concern myself with joint_range_deg
  # But mainly the idea is we have leg can raise laterally and rotate like that, thigh motor, knee motor, and motor in the foot as our wheel.
  # Then we have the neck up and down, then neck pan, and our eyes up and down in sync, so I have to state as actuator turns these also change up and down, of course in like a circular motion. 

  
  hip_x: 0.000                # hip pitch axis, front-back offset from body centre. ESTIMATE
  hip_z: -0.100               # hip pitch axis height rel. body centre (puts body bottom ~5 cm above hip). ESTIMATE
  hip_roll_y: 0.100           # hip-roll axis lateral offset (pitch axis sits at y = track/2). ESTIMATE
  
  #! HEAD
  neck_length: 0.060          # body top → head-tilt axis. ESTIMATE
  left_eye: [0.050, 0.080, 0.250]  # x, y, z rel. head-tilt axis. ESTIMATE
  right_eye: [0.050, -0.080, 0.250]
  eye_offset: 0.020            # lateral distance between eyes. ESTIMATE
  eye_radius: 0.030           # visual only. ESTIMATE
  battery_z: -0.080           # battery height in body (low = more stable). ESTIMATE
  neck_frac: 0.35             # fraction of `head` mass that is the neck pan stage. ESTIMATE
  

  joint_range_deg:            # soft stops for the sim
    hip_roll: [-30, 30]
    hip_pitch: [-120, 60]
    knee: [0, 150]            # 0 = straight leg, positive = knee bends forward
    neck_pan: [-90, 90]       # Neck look around
    neck_UD: [-45, 45]        # Neck up and down
    eyes_UP: [] # THIS HAS TO ROTATE IN A CIRCLE, SO THE EYES move just their farthest most corner up and down. LINKED LIKE AN ORTHOCOPTERS WINGS

  masses:                     # kg, see CLAUDE.md section 3 (all ESTIMATE until weighed)
    leg_actuator: 0.65        # ESTIMATE
    wheel_actuator: 0.33
    wheel: 0.30               # ESTIMATE
    femur: 0.25               # ESTIMATE
    shin: 0.25                # ESTIMATE
    hip_cradle: 0.30          # ESTIMATE
    body_shell: 1.50          # ESTIMATE
    battery: 0.70
    electronics: 0.80         # ESTIMATE
    head: 1.20                # ESTIMATE
    misc: 0.80                # ESTIMATE

electrical:
  bus_voltage: 22.2           # 6S nominal; sweep 21–25

motors:
  m5010_340:  {kv: 340, R: 0.27,  kt: 0.027,  i_peak: 20, i_cont: 6,  tau_peak: 0.54, mass: 0.170, J_rotor: 3.0e-5}  # R verified, kt from prop test; J estimate
  m8108_100:  {kv: 100, R: 0.163, kt: 0.100,  i_peak: 34, i_cont: 11, tau_peak: 3.3,  mass: 0.271, J_rotor: 1.8e-4}  # Hobbywing verified (R, 34.3 A, 271 g); kt from dyno data; i_cont + J estimate
  m6215_170:  {kv: 170, R: 0.071, kt: 0.054,  i_peak: 39, i_cont: 13, tau_peak: 2.08, mass: 0.425, J_rotor: 1.6e-4}  # 2.08 N·m + 425 g verified; R/current scaled from 320KV
  m10_100:    {kv: 100, R: 0.080, kt: 0.0955, i_peak: 63, i_cont: 20, tau_peak: 6.0,  mass: 0.597, J_rotor: 6.8e-4}  # MAD M10 IPE, big-motor reference

actuators:
  hip_roll:  {motor: m8108_100, ratio: 11, eta: 0.85, damping: 0.10, frictionloss: 0.30}
  hip_pitch: {motor: m8108_100, ratio: 11, eta: 0.85, damping: 0.10, frictionloss: 0.30}
  knee:      {motor: m8108_100, ratio: 11, eta: 0.83, damping: 0.10, frictionloss: 0.30}  # extra ~2% for the belt
  wheel:     {motor: m5010_340, ratio: 6,  eta: 0.90, damping: 0.01, frictionloss: 0.03}
  neck_pan:  {motor: m5010_340, ratio: 6,  eta: 0.90, damping: 0.02, frictionloss: 0.03}
  neck_UD:   {motor: m5010_340, ratio: 6,  eta: 0.90, damping: 0.02, frictionloss: 0.02}  # low priority, ESTIMATE
  head_tilt: {motor: sts3215,   ratio: 1,  eta: 1.0,  damping: 0.05, frictionloss: 0.05}  # Feetech STS3215, 2.9 N·m stall @ 12 V
  eye_tilt:  {motor: gm2804,    ratio: 1,  eta: 1.0,  damping: 0.001, frictionloss: 0.002} # ×2, iPower GM2804 gimbal

contact:
  wheel_friction: [1.0, 0.005, 0.0001]   # sliding, torsional, rolling. ESTIMATE
  # TODO: 
  # Note that later in life when this is all proper, stopping and taking walks with this circular wheels I assume would require braking 
  # to add extra friction to the wheels when taking actual steps and not rolling around. 
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
