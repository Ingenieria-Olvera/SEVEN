# The 7: V2 sizing sim (MuJoCo)

Figures out what torque, speed and duty each joint of the V2 wheel-legged droid
needs, so we can pick motor + gear ratio + spring before CAD. See `CLAUDE.md`
for the full brief.

## Install (Windows, PowerShell)

Python is managed with [uv](https://docs.astral.sh/uv/) (already installed on this PC).

```powershell
uv venv --python 3.11 .venv
uv pip install --python .venv\Scripts\python.exe -r requirements.txt
```

Run scripts with the `run` shortcut in the project folder (always works):

```powershell
.\run tutorials\01_basics.py
```

`run.cmd` just calls `.venv\Scripts\python.exe` with whatever you pass it.
In VS Code, `.vscode/settings.json` already points at the venv, so the
Run ▶ button works too.

Or activate the venv so plain `python` uses it. Windows blocks the activate
script by default, so allow it once first:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned   # one time
.venv\Scripts\Activate.ps1                            # prompt now starts with (.venv)
```

`ModuleNotFoundError: No module named 'mujoco'` means you're running a Python
that isn't the venv's: activation failed, or VS Code picked the wrong
interpreter (Ctrl+Shift+P → "Python: Select Interpreter" → `.venv\Scripts\python.exe`).

**Smart App Control gotcha:** Windows 11 Smart App Control is ON here and blocks
DLLs it doesn't recognise. The newest MuJoCo (3.16) fails with
`DLL load failed ... An Application Control policy has blocked this file`.
That's why `requirements.txt` pins `mujoco==3.3.0`, which loads fine.

## Layout

```
config/params.yaml      every tunable number (masses, motors, ratios, sim settings)
tutorials/              learning steps, one concept each
models/                 build_mjcf.py generates robot.xml from params.yaml; view_robot.py
sim/                    actuators.py, experiments/ (e4 jump, e6 hip roll, motor sweep)
results/                plots + summaries, one folder per run (CSVs are git-ignored; re-run to get them)
```

## Steps

### 1. MuJoCo basics: `tutorials/01_basics.py`

```powershell
python tutorials\01_basics.py              # viewer
python tutorials\01_basics.py --headless   # numbers only
```

A box falls onto the floor and a femur-sized arm is held at 45° by a PD
controller. The script prints:

| column | meaning |
|---|---|
| `box z`, `box vz` | from `data.qpos` / `data.qvel`, the box's height and vertical speed |
| `arm deg` | hinge angle from `data.qpos` |
| `ctrl` | what the controller asked for (`data.ctrl`) |
| `act_force` | `data.actuator_force`: actuator output, after clamping to `ctrlrange` |
| `qfrc_act` | `data.qfrc_actuator`: torque that actually lands on the joint (= gear × act_force) |
| `sensor` | the `actuatorfrc` sensor, which is what a real robot's log would show |

Viewer tips: double-click a body to select it, Ctrl + right-drag to push it,
Space pauses, Backspace resets.

### 3. The robot model: `models/build_mjcf.py`, `models/view_robot.py`

```powershell
.\run models\build_mjcf.py              # writes models/robot.xml + prints mass/height report
.\run models\view_robot.py              # body pinned, legs hold the stand pose
.\run models\view_robot.py --free       # body free: falls over (no balance controller yet)
.\run models\view_robot.py --theta 55   # crouch
.\run models\view_robot.py --eyes 20    # eye pods' outer corners up 20°
```

Change the robot by editing `config/params.yaml`, never `robot.xml`.
Sign conventions (match `joint_range_deg` in params.yaml): hip_pitch + = thigh
forward, knee 0 = straight and + = shin swings back, wheel + = rolls forward.
The knees point **backward** (`robot.knee_dir: back`), so the stand pose is
hip_pitch = −θ, knee = −2θ (wheel stays under the hip). `leg_pose(theta, roll)`
in `build_mjcf.py` handles the sign; always pass a positive θ.
Hip roll + swings a wheel toward +y (left) on both legs; use
`hip_roll_pose(lean, spread)` instead of thinking about signs.
Eye pods: one actuator (`eye_tilt`) on eye_L, eye_R = −eye_L via an equality
constraint, so both outer corners move up together.

### Jump: `sim/experiments/e4_jump.py`

```powershell
.\run sim\experiments\e4_jump.py                 # headless: summary + results/e4_jump/<time>/
.\run sim\experiments\e4_jump.py --view          # watch it at 1/4 speed
.\run sim\experiments\e4_jump.py --view --slow 1 # real time
```

Crouch → max-effort push (knee lifts, hip steers to keep the wheel under the
hip) → tuck → land. Motor torque droops with speed (`sim/actuators.py`). The
body rides a vertical test rig (`jump.base: vertical`) so no balance
controller is needed. Each run saves `jump.png`, `jump.csv`, `summary.txt` and a
copy of `params.yaml`.

Knobs in `params.yaml` for "can it jump at X kg with motor Y?":

| question | change |
|---|---|
| heavier / lighter robot | `robot.mass_scale` (check the total with `build_mjcf.py`; 1.0 ≈ 11.5 kg, 1.3 ≈ 15 kg) or individual `robot.masses` |
| different motor | `actuators.knee.motor` / `actuators.hip_pitch.motor` → a name under `motors:` |
| different gearbox | `actuators.knee.ratio` (more torque, less speed) |
| more / less spring | `robot.spring_frac` |
| deeper crouch | `jump.crouch_deg` |
| battery voltage | `electrical.bus_voltage` (raises top speed ω0) |

### Hip roll: `sim/experiments/e6_hip_roll.py`

```powershell
.\run sim\experiments\e6_hip_roll.py               # stand (30°) + crouch (55°), saves results/e6_hip_roll/<time>/
.\run sim\experiments\e6_hip_roll.py --theta 45    # any thigh angles
.\run sim\experiments\e6_hip_roll.py --view        # watch it
```

Hip roll has two jobs, both in `hip_roll_pose(lean, spread)` (`models/build_mjcf.py`):

- **lean**: both legs tilt the same way and the body slides sideways over the
  wheels. This is the carve pose.
- **spread**: the legs tilt opposite ways, giving a wide (+) or narrow (−) stance.

The body rides a test rig (`build(p, base="roll")`): it slides up/down and
sideways and can roll, but it can't pitch, so no balance controller is needed.
The angles ramp slowly, so the torques are holding torques. `hip_roll.png`
plots |torque| against angle with the hand calcs and τ_cont / τ_peak.

What it showed (8108 at 11:1, 11.5 kg):

- **Lean ±15° needs only 3–4.5 N·m**, below CLAUDE.md's F·h·sin φ (8 N·m). The
  wheel sits outboard of the leg, so the outer wheel ends up nearly under its
  hip and carries most of the weight. The same offset rolls the body about 5°.
- **Changing stance width under load scrubs the wheels sideways.** Friction
  ≈ μ·F·h ≈ 30 N·m, which saturates the motor at θ = 30°. The torque depends on
  the path the legs took (the loops in the plot). μ = `contact.wheel_friction[0]`
  is an ESTIMATE. Change stance slowly, while rolling, or with a leg unloaded.
- Narrow −15° at stand puts the wheels on top of each other (track 7 cm).

### Motor sweep: `sim/experiments/sweep_motors.py`

```powershell
.\run sim\experiments\sweep_motors.py          # 208 cases, ~15 s on 16 cores
.\run sim\experiments\sweep_motors.py --quick  # only sweep.mass_scale from params.yaml
```

For every `sweep.leg_motor` × `sweep.ratio` × mass, it runs the jump (e4), a 15° lean (e6)
and a one-leg stand at 30° and 45°. `summary.md` gives the **heaviest robot each combo can
be** while passing:

- **sustained holds** (2-leg stand, 60° crouch, 15° lean) ≤ τ_cont
- **jump wheel clearance** ≥ 20 / 30 / 40 cm (target not decided, so all three are reported)
- **one-leg stand** (stretch): not saturated, sag < 10°

`sweep.csv` has every number, including `body_rise_cm`. Clearance = body rise +
how far the legs tuck up in the air, and the tuck is usually more than half of it.

## Adding a link or joint

The model is a tree in `models/build_mjcf.py`. Every part is a `<body>`; the
`<joint>` inside a body connects it to its parent. To add one (e.g. hip yaw
between the body and the hip cradle):

1. **Nest a new `<body>`** where it sits in the chain, with `pos` = where its
   joint axis is, measured in the parent's frame.
2. **Give it a `<joint>`** (type `hinge` + `axis`, or `slide`), a `range`, and
   armature/damping/friction via `joint_xml(...)`.
3. **Give it an `<inertial>`** (`inertial([...parts])`) and a `<geom>` to see it.
4. **Add a `<motor>`** in the actuator list and an entry under `actuators:` in
   `params.yaml`.
5. **Add its value to the `stand` keyframe** in the right tree order (the
   number of values must equal `nq`, or MuJoCo will refuse to load).

Closed loops (4-bars, parallel linkages) can't be a tree. Build one side as a
chain, then close the loop with an `<equality><connect .../></equality>`
constraint. Belts and gears (one joint driving another at a fixed ratio) use
`<equality><joint joint1=".." joint2=".." polycoef="0 ratio 0 0 0"/>` or a
`<tendon><fixed>`.
