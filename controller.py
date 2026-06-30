"""
controller.py
------------------------------------------------------------
Continuous LQR Controller

Author : TaeWoo + ChatGPT
"""

from __future__ import annotations

import numpy as np

from scipy.linalg import solve_continuous_are

from aircraft import Aircraft
from control import ControlInput


class LQRController:
    """
    Continuous-time LQR Controller.
    """

    def __init__(self, aircraft: Aircraft):

        self.aircraft = aircraft

        self.A = aircraft.A
        self.B = aircraft.B

        # ===============================================
        # LQR Weight Matrices
        # ===============================================

        self.Q = np.diag([

            100.0,     # Roll

            10.0,      # Roll Rate

            150.0,     # Pitch

            15.0,      # Pitch Rate

            80.0,      # Yaw

            8.0        # Yaw Rate

        ])

        self.R = np.diag([

            0.8,

            0.5,

            1.0

        ])

        # ===============================================

        self.max_aileron = np.deg2rad(25)

        self.max_elevator = np.deg2rad(25)

        self.max_rudder = np.deg2rad(30)

        self.max_rate = np.deg2rad(60)

        self.K = self.compute_gain()

        self.previous_control = ControlInput()

    # ===================================================
    # Gain
    # ===================================================

    def compute_gain(self):

        """
        Solve CARE

        A'P+PA-PBR^-1B'P+Q=0
        """

        P = solve_continuous_are(

            self.A,

            self.B,

            self.Q,

            self.R

        )

        K = np.linalg.solve(

            self.R,

            self.B.T @ P

        )

        return K

    # ===================================================
    # Update Gain
    # ===================================================

    def update_gain(self):

        self.K = self.compute_gain()

    # ===================================================
    # Set Weight Matrix
    # ===================================================

    def set_Q(self, Q):

        self.Q = Q

        self.update_gain()

    def set_R(self, R):

        self.R = R

        self.update_gain()

    # ===================================================
    # Reference Error
    # ===================================================

    @staticmethod
    def state_error(

        state,

        reference

    ):

        state = np.asarray(

            state,

            dtype=float

        ).reshape(6)

        reference = np.asarray(

            reference,

            dtype=float

        ).reshape(6)

        return state-reference

    # ===================================================
    # Saturation
    # ===================================================

    def saturation(

        self,

        control

    ):

        control = np.asarray(

            control,

            dtype=float

        ).reshape(3)

        control[0] = np.clip(

            control[0],

            -self.max_aileron,

            self.max_aileron

        )

        control[1] = np.clip(

            control[1],

            -self.max_elevator,

            self.max_elevator

        )

        control[2] = np.clip(

            control[2],

            -self.max_rudder,

            self.max_rudder

        )

        return control
    # ===================================================
    # Rate Limiter
    # ===================================================

    def rate_limit(
        self,
        control: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Apply first-order rate limiting to control surfaces.
        """

        previous = self.previous_control.to_vector()

        maximum_change = self.max_rate * dt

        delta = control - previous

        delta = np.clip(
            delta,
            -maximum_change,
            maximum_change
        )

        limited = previous + delta

        return limited

    # ===================================================
    # Control Law
    # ===================================================

    def control(
        self,
        state: np.ndarray,
        reference: np.ndarray,
        dt: float,
    ) -> ControlInput:
        """
        Continuous LQR Control

        u = -K(x-r)
        """

        error = self.state_error(
            state,
            reference
        )

        control = -(self.K @ error)

        control = self.saturation(control)

        control = self.rate_limit(
            control,
            dt
        )

        output = ControlInput.from_vector(
            control
        )

        self.previous_control = output.copy()

        return output

    # ===================================================
    # Reset
    # ===================================================

    def reset(self):

        self.previous_control = ControlInput()

    # ===================================================
    # Gain Information
    # ===================================================

    def gain(self):

        return self.K.copy()

    # ===================================================
    # Summary
    # ===================================================

    def summary(self):

        print("\n" + "=" * 60)

        print("LQR Controller")

        print("=" * 60)

        print()

        print("Q Matrix")

        print(self.Q)

        print()

        print("R Matrix")

        print(self.R)

        print()

        print("Gain Matrix")

        print(self.K)

        print()

        print("Surface Limits")

        print(
            f"Aileron  : ±{np.rad2deg(self.max_aileron):.1f} deg"
        )

        print(
            f"Elevator : ±{np.rad2deg(self.max_elevator):.1f} deg"
        )

        print(
            f"Rudder   : ±{np.rad2deg(self.max_rudder):.1f} deg"
        )

        print()

        print(
            f"Rate Limit : {np.rad2deg(self.max_rate):.1f} deg/s"
        )

    # ===================================================
    # String
    # ===================================================

    def __repr__(self):

        return "LQRController()"


# ==========================================================
# Standalone Test
# ==========================================================

if __name__ == "__main__":

    aircraft = Aircraft()

    controller = LQRController(
        aircraft
    )

    controller.summary()

    state = np.array([

        np.deg2rad(10),

        0.0,

        np.deg2rad(5),

        0.0,

        np.deg2rad(3),

        0.0

    ])

    reference = np.zeros(6)

    control = controller.control(

        state,

        reference,

        dt=0.01

    )

    print()

    print("Control Command")

    print(control)

    print()

    print("Vector")

    print(control.to_vector())