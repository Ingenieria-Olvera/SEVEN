# CLAUDE.md: The 7, V2 wheel-legged droid, MuJoCo sizing sim

This file briefs a Claude Code agent working with Jonny, a UMass Amherst student building **The 7**: a swarm of 7 personality robots for swarm-robotics research. This repo is a **MuJoCo sizing simulation** for the V2 robot. Its job is to answer one question before CAD starts:

> **What torque, speed and duty does each joint actually need for the moves we want, and which motor + gear ratio + spring combination gets us there at the lowest mass and cost?**

Every number below is a design estimate unless it's marked **[verified]**. The sim is how we replace estimates with evidence.

---

## 0. How to work with Jonny (read first)

- **Teach while building.** Jonny is new to MuJoCo. Before each step, say in 2–3 sentences what you're about to build and why. After it runs, explain what the output means. He wants to be able to run and change things **without** an AI later.
- **Small steps, each one runnable.** Build in the order in section 6. Don't jump to the full robot or to RL. Every step ends with something he can open in the viewer or a plot he can read.
- **Let him drive sometimes.** At key points (changing a mass, adding a joint, writing a trajectory), offer to let him write it and review it instead of writing it yourself.
- **Peer-level, plain language.** Use section headers, short bullets and real numbers. Define a term the first time it appears (e.g. "armature = motor rotor inertia as felt at the joint").
- **Keep parameters in one place.** Everything tunable lives in `config/params.yaml` (section 5). Scripts read from it, so a sweep never means editing code.
- **Flag guesses.** If you invent a value (rotor inertia, friction), mark it `# ESTIMATE` in the YAML and say so in chat.
- **Platform:** Jonny works on Windows with Fusion 360. Use Python 3.11 in a venv, and paths that work on Windows (`pathlib`).

---

## 1. The robot (V2 concept)

A Disney-BDX-style droid: a body ("stomach") with a head and twin eye cameras, and two legs that end in **powered wheels** instead of feet. No arms. It balances on its wheels like a Segway, and its legs change height, lean and jump.

### Target size
| Item | Value |
|---|---|
| Overall height | about 1.0 m (0.8 m is acceptable) |
| Eye height | about 95 cm (hip height of a 6'0" person) |
| Hip-pitch axis height, standing | about 56 cm |
| Thigh (femur) / shin | 280 mm / 280 mm, equal links |
| Wheel | Ø160 mm |
| Body (L × W × H) | about 270 × 380 × 310 mm |
| Track (wheel centre to centre) | about 300 mm |
| Mass | design for **15 kg max**, aim for **12 kg**; sweep 10–16 kg |

### Joints per leg (×2), plus head
| Joint | Axis | Lives | Actuator (current plan) |
|---|---|---|---|
| Hip roll | front–back (x) | body, behind the hip cradle; front support bearing | leg actuator (cycloidal) |
| Hip pitch | side–side (y) | hip cradle, outboard | leg actuator (cycloidal) |
| Knee | side–side (y) | hip cradle, inboard, coaxial with hip pitch; drives the knee through a **timing belt** down the femur | leg actuator (cycloidal) |
| Wheel | side–side (y) | wheel hub | 5010 + 6:1 planetary (already built) |
| Neck pan / head tilt | z / y | top of body | 5010 + 6:1, or a small servo (low priority) |

**Belt coupling:** knee motor angle = hip pitch angle + knee angle (1:1 belt). In sim, start by putting the knee actuator **directly on the knee joint**, then add the coupling in a later step (section 6, step 7).

**"Wheel under hip":** with equal links, the wheel stays under the hip when hip pitch = −½ × knee angle. Hip height = 2·L·cos(θ), where θ = hip pitch angle from vertical.

**Knee spring:** a torsion spring on the knee shaft that pushes the leg straight. Model it as a torsional spring on the knee joint, with stiffness chosen to carry a fraction `spring_frac` of body weight at the stand pose.

### Goals (what the sim must test)
1. Stand, crouch and change height smoothly (core).
2. Balance on two wheels and recover from pushes (core).
3. **Jump**: target to be confirmed by Jonny (20 / 30 / 40 cm clearance).
4. **Carve turns** with hip roll (lean into turns), wide/narrow stance (core-plus).
5. **One-leg stand / one-leg skating**: stretch goal. A single wheel is unstable side to side, so it likely needs hip yaw or using the free leg as a counterweight. Test feasibility and say honestly whether it works.
6. Survive landings: peak joint torque and speed at touchdown.

Swarm-research scope comes first. This robot will be built 7 times, so cost per joint matters.

---

## 2. Actuator candidates and the math behind them

### Core formulas
- Torque constant: **Kt ≈ 9.55 / KV** [N·m/A]. A rough approximation, expect ±15%; replace it with measured values when available.
- Motor torque: τ_motor = Kt · I
- Output torque: **τ_out = τ_motor · N · η**, where N = gear ratio and η = efficiency
- No-load motor speed: ω_m0 = KV · V · 2π / 60 [rad/s]; output: **ω_out0 = ω_m0 / N**
- Torque–speed limit (simple linear droop): **τ_max(ω) = τ_peak · (1 − |ω| / ω_out0)**, clamped ≥ 0. Use this in the sim, not a flat torque limit.
- Reflected rotor inertia at the joint: **J_joint = J_rotor · N²**, which goes into MJCF `armature`.
- Copper loss (for heating): P = I² · R_phase · 1.5 (approx). Use the RMS current over a routine to check continuous ratings.
- Battery: 6S LiPo, nominal 22.2 V (sweep 21–25 V).

### Candidates
| ID | Motor | KV | Kt (est.) | Peak motor torque | Continuous motor torque | Gearbox | η | Notes |
|---|---|---|---|---|---|---|---|---|
| `wheel_5010_6` | 5010 (AliExpress, $13.63) | 340 | 0.028 | 0.42 N·m @ 15 A (0.56 @ 20 A burst) | 0.14 N·m @ 5 A | 6:1 printed planetary (sun 15 / planets 30 / ring 75, m1) | 0.90 | **Built** (on Jonny's resume). Output ≈ 2.2 N·m peak, 0.75 continuous, ω_out0 ≈ 132 rad/s @ 22.2 V. Planned **wheel drive**. R ≈ 0.27 Ω. Mass ≈ 170 g motor, about 330 g actuator. |
| `leg_8108_c11` | Hobbywing XRotor M8108 | 100 | 0.096 | about 2.1 N·m (MAD's 8108 benchmark; ESTIMATE for Hobbywing) | about 0.8 N·m (ESTIMATE) | cycloidal **11:1** | 0.85 | Ø87 pancake, $105, IP45. Output ≈ 19.6 N·m peak, 7.5 continuous. ω_out0 ≈ 21 rad/s. |
| `leg_8108_c15` | same | 100 | 0.096 | 2.1 | 0.8 | cycloidal 15:1 | 0.83 | About 26 N·m peak, 10 continuous, ω_out0 ≈ 15 rad/s. Less backdrivable. |
| `leg_6215_c11` | MAD 6215 IPE | 170 | 0.056 | 1.7–2.2 N·m @ 30–40 A (ESTIMATE) | about 0.67 N·m @ 12 A (ESTIMATE) | cycloidal 11:1 | 0.85 | **[verified]** Ø70 × 37 mm, 6 mm shaft, IP35, 14 pole pairs, $98, mount 4× M3 on Ø30, rotor 4× M3 on Ø12 and Ø18. Weight, R and max current not published. Berkeley Humanoid Lite uses the sibling M6C12 150 KV. Taller motor → wider hip stack. |
| `leg_big_c11` | 10015-class placeholder | sweep 60–100 | — | 3–5 N·m (ESTIMATE) | 1.2–2 N·m | cycloidal 11:1 | 0.85 | Placeholder to find what the moves *require*. Look for real motors after. |

**Rotor inertia (ESTIMATE until measured):** J_rotor ≈ ½ · m_rotor · r². Use 8108 ≈ 1.2e-4 kg·m², 6215 ≈ 6e-5, 5010 ≈ 2e-5. At 11:1, the 8108's armature ≈ 0.0145 kg·m².

### Cycloidal design already decided (for context, not simulated as gears)
11:1 (11 lobes, 12 ring pins), ring-pin circle Ø90, eccentric 2.0 mm, ring pins and output pins are **623ZZ rollers** (3 × 10 × 4) on 3 mm steel dowels, output holes Ø14 on a Ø54 circle, cam bearings 6804, main output bearing 6811, two discs 180° apart (disc 2 lobes clocked 180°/11 ≈ 16.36°). Actuator envelope about Ø112. **Do not model gear teeth in MuJoCo.** Model each actuator as a torque source with torque–speed limits, armature, damping and friction.

### Hand-calc requirements so far (the sim should confirm or replace these)
Knee torque per leg ≈ F · L · sin θ, where F = (1 − spring_frac) · m · g / n_legs_loaded and L = 0.28 m.

| Pose | 12 kg | 15 kg |
|---|---|---|
| Two legs, stand θ = 30° | 8.2 N·m | 10.3 |
| Two legs, crouch θ = 55° | 13.5 | 16.9 |
| One leg, stand θ = 30° | 16.5 | 20.6 |
| One leg, crouch θ = 45° | 23.3 | 29.1 |
| One leg, near straight θ = 15° | 8.5 | 10.7 |
| Jump push-off (about 3× two-leg crouch) | about 40 | about 50 |

- Hip roll at tilt φ: τ ≈ F · h · sin φ (h ≈ hip height). For 15°, 12 kg: ≈ 7.6 N·m.
- Wheels: to catch a lean of angle α the base must accelerate a ≈ g · tan α; force = 2 · τ_wheel / r_wheel. A bare 5010 gives about 0.9 m/s² at 12 kg (not enough); the 5010 + 6:1 gives about 4.6 m/s².
- **Jonny's robotics friend says 18 N·m peak isn't enough for the jump and one-leg goals.** The table agrees. Use the sim to find the real numbers.

---

## 3. Mass budget (starting point, every line is sweepable)

Per-part masses live in `robot.masses` in `config/params.yaml`; `models/build_mjcf.py` prints the per-body breakdown and total (about 11.5 kg at `mass_scale: 1.0`).

Put each mass at a reasonable centre of mass position in the MJCF. The leg actuators sit in the **body/hip cradle**, not on the legs, so the legs are light. That's the point of the belt-driven knee.

---

## 4. Repo layout to create

```
the7-sim/
  CLAUDE.md                 # this file
  README.md                 # how to install + run each step (write for Jonny)
  requirements.txt          # mujoco, numpy, pyyaml, matplotlib, pandas
  config/
    params.yaml             # ALL tunables (section 5)
  models/
    build_mjcf.py           # generates robot.xml from params.yaml (no hand-edited XML)
    robot.xml               # generated, do not edit by hand
  sim/
    actuators.py            # torque–speed clamp, motor/ratio lookup, current & heat estimates
    controllers.py          # PD joint control, wheel balance (LQR or PD), pose interpolation
    experiments/
      e0_sanity.py
      e1_static_sweep.py
      e2_one_leg.py
      e3_balance_push.py
      e4_jump.py
      e5_landing.py
      e6_carve_roll.py
      e7_one_wheel.py       # stretch
      e8_duty_thermal.py
  results/                  # CSVs + PNG plots, one folder per run with the params snapshot
  notebooks/                # optional, for Jonny to poke at results
```

Setup and running: see README (`uv` venv, `.\run <script>`; mujoco pinned to 3.3.0 because Windows Smart App Control blocks newer MuJoCo DLLs).

---

## 5. `config/params.yaml`

All tunables live in `config/params.yaml` (tags: VERIFIED / DERIVED / ESTIMATE). It is the source of truth; read it rather than assuming values.

---

## 6. Build order (each step runnable, each one explained to Jonny)

1. **MuJoCo basics (about 30 min).** Explain MJCF: `worldbody`, `body`, `joint`, `geom`, `inertial`, `actuator`, `sensor`, and `option timestep`. Drop a box on a plane, then open it with `mujoco.viewer.launch_passive`. Show `data.qpos`, `data.qvel`, `data.actuator_force`, `data.qfrc_actuator`.
2. **One leg on a fixed hip (`e0_sanity`).** Femur + shin + wheel hanging from a fixed point. Add motor-type actuators with `gear`, `ctrlrange` and `forcerange`, plus `armature`, `damping` and `frictionloss` on the joints. Run PD position control to a few poses and plot torque.
3. **`build_mjcf.py`.** Generate the full robot XML from `params.yaml`: body, 2 legs with roll/pitch/knee/wheel, wheel cylinders with contact, head. Check the total mass printed by `mujoco.mj_getTotalmass` matches the budget.
4. **`actuators.py`.** Apply torque through `data.ctrl`, clamped every step by τ_max(ω) using the formulas in section 2. Log required torque, clamped torque, joint speed and estimated motor current. Saturation must be visible in the logs: a plot that shows "the motor wanted 30 but could give 19" is the most useful output in the repo.
5. **Static sweeps (`e1`, `e2`).** Hold the body still and set poses θ = 10…70° for mass 10–16 kg and spring_frac 0–0.5. Record the joint torques needed. Compare against the hand-calc table in section 2. If they disagree by more than 15%, investigate and explain why.
6. **Balance (`e3`).** Wheel-inverted-pendulum balance controller (LQR on a linearised model, or tuned PD on pitch + wheel velocity). Push tests: an impulse to the body, and measure wheel torque/speed needed to recover. Report the largest push survived for each wheel actuator option.
7. **Belt coupling.** Re-route the knee actuator through the hip so that knee motor angle = hip + knee (use a `fixed` tendon or compute it in the controller). Show how much the hip and knee torques shift.
8. **Jump (`e4`).** Crouch, then a maximum-effort extension with the torque–speed clamp. Measure apex clearance. Sweep ratio {10, 11, 15}, motor {8108, 6215, big} and mass. Output a table: config → jump height, peak torque, peak speed, saturated? (y/n).
9. **Landing (`e5`).** Drop from 20/30/40 cm. Record peak joint torque, speed and impact current. A backdrivable joint should comply instead of hitting a hard limit. Report whether the current limit is exceeded.
10. **Carve and roll (`e6`).** Drive in an arc while leaning with hip roll ±15°. Record roll torque and lateral tip-over margin.
11. **One-wheel stand (`e7`, stretch).** Lift one leg. Test whether hip roll + swinging the free leg can balance side to side on one wheel. If it can't, say so, and test adding a hip-yaw joint. Report honestly; this decides whether a 5th joint is worth it.
12. **Duty / heat (`e8`).** Run a 60 s "show routine" (stand, crouch, carve, jump, land). Compute RMS motor current per joint vs `i_cont`. Any joint above its continuous rating needs a bigger motor, a higher ratio or more spring.

**Final deliverable:** `results/requirements.md`, with a table per joint giving peak torque, RMS torque, peak speed and the required continuous torque, plus which candidate passes each test. That table picks the motor.

---

## 7. MuJoCo modelling notes

- **Units:** SI throughout (m, kg, s, N·m). Fusion is in mm; convert once at the edge.
- **Actuators:** use `<motor joint="..." gear="1" ctrllimited="true" ctrlrange="-τpeak τpeak"/>` and apply the torque–speed clamp in Python each step. Put the reflected rotor inertia in the joint's `armature`.
- **Wheels:** a cylinder geom with its axis along y. Use `condim="3"` (or 4–6 if you need rolling/torsional friction). Keep the timestep at 1 ms or less for stable contact.
- **Spring:** knee joint `stiffness` + `springref`, chosen so the spring torque at the stand pose = spring_frac × the required knee torque.
- **Inertials:** give each body an explicit `<inertial>` with mass and a box-like diagonal inertia. Don't rely on geom density for printed parts.
- **Sensors:** `framequat`/`gyro`/`accelerometer` on the body (IMU), plus `jointpos`/`jointvel`/`actuatorfrc`.
- **Don't model:** gear teeth, cycloid lobes, belts as flexible parts, or PCB detail. These are sizing sims, not mechanism sims.
- **Later (not now):** export the CAD to URDF/MJCF with real inertias, and move to Isaac Sim / Isaac Lab (or MuJoCo Playground) for RL locomotion, the way Open Duck Mini v2 and Berkeley Humanoid Lite do.

---

## 8. References

- **Berkeley Humanoid Lite** (16 kg, 0.8 m, 3D-printed cycloidal actuators, MAD M6C12 150 KV, 6811 output bearing, B-G431B-ESC1 driver): arXiv 2504.17249
- **Ascento** (10.4 kg wheel-legged, about 40 N·m peak, springs, about 0.4 m jump): arXiv 2005.11435
- **Upkie** (open-source wheeled biped, hip + knee + wheel)
- **DIABLO** (22.9 kg wheel-legged, 140 mm links)
- **MIT Mini Cheetah** (6:1 planetary QDD, 17 N·m peak; roll in body, pitch + knee at hip, belt knee)
- **Open Duck Mini v2** (BDX-style, MuJoCo Playground pipeline)
- MuJoCo docs: https://mujoco.readthedocs.io (MJCF reference, `mujoco.viewer`, actuators, contact)

---

## 9. Open questions to raise with Jonny (don't decide these silently)

1. Jump target height (20 / 30 / 40 cm)?
2. One-leg stand / skating: requirement or stretch goal?
3. Mass ceiling: 15 kg OK?
4. Budget per robot for actuators (×7 robots)?
5. Is a hip-yaw joint (5 per leg) acceptable if `e7` shows one-wheel balance needs it?
6. Real 8108 height and weight from the STEP/datasheet, and MAD 6215 weight/R/max current: replace the ESTIMATEs as soon as they're known.
