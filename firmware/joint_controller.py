"""Per-actuator firmware model.

Each EC-series actuator runs a PD position loop with the documented limits:
torque clamped to peak, velocity fold-back at the datasheet limit, and an
electrical power estimate from the torque constant Kt.
"""
WINDING_R = 0.3   # ohms, assumed phase resistance (not published)
DRIVER_IDLE_W = 1.5  # quiescent draw per driver board


class JointController:
    def __init__(self, spec, kp=None, kd=None):
        self.spec = spec
        self.name = spec["joint"]
        self.peak = spec["peak_torque"]
        self.rated = spec["rated_torque"]
        self.vel_limit = spec["vel_limit"]
        self.kt = spec["kt"]
        # stance joints run stiff position loops (weight-bearing joints sag
        # visibly at low gain, distorting squat geometry); arms stay softer
        leg = any(k in spec["joint"] for k in ("hip", "knee", "ankle", "waist"))
        kp_scale = 10.0 if leg else 5.0
        self.kp = kp if kp is not None else kp_scale * self.rated
        self.kd = kd if kd is not None else self.kp / 20.0
        self.q_cmd = 0.0   # commanded target (may step)
        self.q_des = 0.0   # ramped setpoint fed to the PD loop
        self.ramp_rate = 0.5 * self.vel_limit  # trajectory slew limit
        self.overload_time = 0.0  # seconds spent above rated torque (thermal proxy)

    def torque(self, q, dq, dt, tau_ff=0.0):
        # slew-rate limit the setpoint: stepping targets straight into the PD
        # loop spikes phase current past the BMS limit (found in stage 4)
        step = max(-self.ramp_rate * dt, min(self.ramp_rate * dt, self.q_cmd - self.q_des))
        self.q_des += step
        # tau_ff: gravity feed-forward from the dynamics model; without it the
        # PD loop droops ~0.1 rad holding a leg horizontal (found in stage 2)
        tau = tau_ff + self.kp * (self.q_des - q) - self.kd * dq
        # velocity fold-back: no accelerating torque past the datasheet limit
        if dq > self.vel_limit and tau > 0:
            tau = 0.0
        elif dq < -self.vel_limit and tau < 0:
            tau = 0.0
        tau = max(-self.peak, min(self.peak, tau))
        if abs(tau) > self.rated:
            self.overload_time += dt
        else:
            self.overload_time = max(0.0, self.overload_time - dt)
        return tau

    def power(self, tau, dq):
        """Electrical power drawn from the bus (regen not credited)."""
        mech = max(0.0, tau * dq)
        current = abs(tau) / self.kt
        return mech + current * current * WINDING_R + DRIVER_IDLE_W


class AnkleRSU:
    """Parallel RSU ankle kinematics: two identical A/B motors drive pitch
    and roll through pushrods.

    Docs give theta_p = (theta_A - theta_B) * r / (2d); the roll relation is
    assumed symmetric with lever arm w. The spec table already expresses
    torque limits in joint space, so control runs in joint space; this map
    exists to verify the motor stroke covers the joint envelope and to report
    motor-side angles like the real firmware does.
    """

    def __init__(self, r=0.025, d=0.055, w=0.045):
        self.kp_dif = r / (2 * d)   # pitch per unit motor differential
        self.kr_sum = r / (2 * w)   # roll per unit motor common mode

    def motor_angles(self, theta_p, theta_r):
        half_dif = theta_p / (2 * self.kp_dif)
        half_sum = theta_r / (2 * self.kr_sum)
        return half_sum + half_dif, half_sum - half_dif  # theta_A, theta_B

    def joint_angles(self, theta_a, theta_b):
        return ((theta_a - theta_b) * self.kp_dif,
                (theta_a + theta_b) * self.kr_sum)
