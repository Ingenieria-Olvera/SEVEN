"""Actuator model (CLAUDE.md section 2): torque–speed limit, current and heat estimates.

MuJoCo's <motor> is an ideal torque source. Real motors lose torque as they
spin up (back-EMF eats the bus voltage), so every physics step we pass the
controller's torque request through Actuator.limit() before it reaches data.ctrl.

Two kinds of motor in params.yaml:
  * BLDC (default): kv, kt, R, i_peak, i_cont, optional tau_peak → output via ratio and eta
  * type: servo / gimbal: datasheet tau_peak, tau_cont, speed (rad/s) used directly
"""
import math

# Reflected inertia for motors with no J_rotor in params.yaml. ESTIMATE:
# a geared servo's internal ~300:1 gearbox makes its rotor feel heavy at the output.
DEFAULT_J_OUT = {"servo": 1e-3, "gimbal": 5e-6, "bldc": 0.0}


class Actuator:
    def __init__(self, motor, ratio, eta, bus_voltage):
        self.kind = motor.get("type", "bldc")
        self.N, self.eta = ratio, eta
        self.mass = motor.get("mass")                         # motor only, kg
        self.R = motor.get("R")
        if self.kind == "bldc":
            self.kt = motor.get("kt", 9.55 / motor["kv"])     # N·m/A at the motor
            self.i_peak, self.i_cont = motor["i_peak"], motor["i_cont"]
            tau_m_peak = motor.get("tau_peak", self.kt * self.i_peak)   # dyno value wins if given
            self.tau_peak = tau_m_peak * ratio * eta          # at the output
            self.tau_cont = self.kt * self.i_cont * ratio * eta
            self.w0 = motor["kv"] * bus_voltage * 2 * math.pi / 60 / ratio   # no-load output speed, rad/s
            self.armature = motor.get("J_rotor", 0.0) * ratio * ratio
        else:
            self.kt = self.i_peak = self.i_cont = None
            self.tau_peak = motor["tau_peak"] * ratio * eta
            self.tau_cont = motor["tau_cont"] * ratio * eta
            self.w0 = motor["speed"] / ratio
            self.armature = motor.get("J_rotor", DEFAULT_J_OUT[self.kind]) * ratio * ratio

    def tau_max(self, tau_cmd, w):
        """Available torque in the requested direction at joint speed w.

        Motoring (torque pushes the way the joint is already moving): droops
        linearly to 0 at w0. Braking (torque opposes motion): back-EMF helps,
        so full peak torque is available.
        """
        if tau_cmd * w > 0:
            return self.tau_peak * max(0.0, 1.0 - abs(w) / self.w0)
        return self.tau_peak

    def limit(self, tau_cmd, w):
        t = self.tau_max(tau_cmd, w)
        return max(-t, min(t, tau_cmd))

    def current(self, tau_out):
        """Motor phase current needed for an output torque, A (nan for servos)."""
        if self.kt is None:
            return float("nan")
        return abs(tau_out) / (self.kt * self.N * self.eta)

    def copper_loss(self, tau_out):
        """Heat in the windings, W (approx: 1.5 · I² · R_phase)."""
        i = self.current(tau_out)
        return 1.5 * i * i * self.R


def load_actuators(p):
    """{joint type: Actuator}, e.g. out['knee'].tau_peak."""
    V = p["electrical"]["bus_voltage"]
    return {j: Actuator(p["motors"][a["motor"]], a["ratio"], a["eta"], V)
            for j, a in p["actuators"].items()}
