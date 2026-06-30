"""
aircraft.py
------------------------------------------------------------
Aircraft Mathematical Model

NASA GTM Inspired (Simplified)

Author : TaeWoo + ChatGPT
"""

from __future__ import annotations

import numpy as np

from dataclasses import dataclass


@dataclass(slots=True)
class AircraftParameters:
    """
    Simplified transport aircraft parameters.
    """

    mass: float = 63500.0

    gravity: float = 9.81

    wing_area: float = 124.6

    wing_span: float = 35.8

    mean_chord: float = 4.29

    cruise_speed: float = 230.0

    cruise_altitude: float = 10000.0


class Aircraft:

    """
    Linear aircraft model

    x_dot = A x + B u
    """

    def __init__(self):

        self.parameters = AircraftParameters()

        self.A = self._create_state_matrix()

        self.B = self._create_input_matrix()

        self.C = np.eye(6)

        self.D = np.zeros((6, 3))

    # =======================================================
    # State Matrix
    # =======================================================

    def _create_state_matrix(self):

        """
        Build linear state matrix.

        State

        x

        roll
        roll_rate

        pitch
        pitch_rate

        yaw
        yaw_rate
        """

        return np.array(

            [

                [
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                ],

                [
                    -0.46,
                    -1.15,
                    0.02,
                    0.00,
                    0.00,
                    0.08
                ],

                [
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0
                ],

                [
                    0.03,
                    0.00,
                    -0.61,
                    -1.52,
                    0.00,
                    0.00
                ],

                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0
                ],

                [
                    0.01,
                    0.07,
                    0.00,
                    0.00,
                    -0.26,
                    -0.91
                ]

            ],

            dtype=float

        )

    # =======================================================
    # Control Matrix
    # =======================================================

    def _create_input_matrix(self):

        """
        Build control matrix.

        Inputs

        aileron

        elevator

        rudder
        """

        return np.array(

            [

                [
                    0.0,
                    0.0,
                    0.0
                ],

                [
                    0.92,
                    0.00,
                    0.09
                ],

                [
                    0.0,
                    0.0,
                    0.0
                ],

                [
                    0.00,
                    1.34,
                    0.00
                ],

                [
                    0.0,
                    0.0,
                    0.0
                ],

                [
                    0.08,
                    0.00,
                    0.84
                ]

            ],

            dtype=float

        )

    # =======================================================
    # Dynamics
    # =======================================================

    def dynamics(

        self,

        state,

        control,

        disturbance=None

    ):

        """
        Continuous dynamics

        x_dot = Ax + Bu + w
        """

        state = np.asarray(

            state,

            dtype=float

        ).reshape(6)

        control = np.asarray(

            control,

            dtype=float

        ).reshape(3)

        if disturbance is None:

            disturbance = np.zeros(6)

        disturbance = np.asarray(

            disturbance,

            dtype=float

        ).reshape(6)

        return (

            self.A @ state

            +

            self.B @ control

            +

            disturbance

        )

    # =======================================================
    # Output
    # =======================================================

    def output(

        self,

        state,

        control=None

    ):

        """
        y = Cx + Du
        """

        state = np.asarray(

            state,

            dtype=float

        ).reshape(6)

        if control is None:

            control = np.zeros(3)

        control = np.asarray(

            control,

            dtype=float

        ).reshape(3)

        return (

            self.C @ state

            +

            self.D @ control

        )
    # =======================================================
    # Trim State
    # =======================================================

    @staticmethod
    def trim_state() -> np.ndarray:
        """
        Aircraft trim state.

        Level flight.
        """

        return np.zeros(6, dtype=float)

    # =======================================================
    # Trim Control
    # =======================================================

    @staticmethod
    def trim_control() -> np.ndarray:
        """
        Trim control input.
        """

        return np.zeros(3, dtype=float)

    # =======================================================
    # Eigenvalues
    # =======================================================

    def eigenvalues(self) -> np.ndarray:
        """
        Open-loop eigenvalues.
        """

        return np.linalg.eigvals(self.A)

    # =======================================================
    # Stability
    # =======================================================

    def is_stable(self) -> bool:
        """
        Check open-loop stability.
        """

        eig = self.eigenvalues()

        return np.all(np.real(eig) < 0)

    # =======================================================
    # Controllability
    # =======================================================

    def controllability_matrix(self) -> np.ndarray:

        n = self.A.shape[0]

        matrix = self.B.copy()

        current = self.B.copy()

        for _ in range(1, n):

            current = self.A @ current

            matrix = np.hstack((matrix, current))

        return matrix

    # =======================================================
    # Observability
    # =======================================================

    def observability_matrix(self) -> np.ndarray:

        n = self.A.shape[0]

        matrix = self.C.copy()

        current = self.C.copy()

        for _ in range(1, n):

            current = current @ self.A

            matrix = np.vstack((matrix, current))

        return matrix

    # =======================================================
    # Rank
    # =======================================================

    def controllability_rank(self) -> int:

        return np.linalg.matrix_rank(
            self.controllability_matrix()
        )

    def observability_rank(self) -> int:

        return np.linalg.matrix_rank(
            self.observability_matrix()
        )

    # =======================================================
    # Validation
    # =======================================================

    def validate(self):

        if self.A.shape != (6, 6):

            raise ValueError(
                "State matrix must be 6x6."
            )

        if self.B.shape != (6, 3):

            raise ValueError(
                "Input matrix must be 6x3."
            )

        if self.C.shape != (6, 6):

            raise ValueError(
                "Output matrix must be 6x6."
            )

        if self.D.shape != (6, 3):

            raise ValueError(
                "Feedthrough matrix must be 6x3."
            )

        if self.controllability_rank() != 6:

            raise RuntimeError(
                "Aircraft model is uncontrollable."
            )

        if self.observability_rank() != 6:

            raise RuntimeError(
                "Aircraft model is unobservable."
            )

    # =======================================================
    # Rebuild
    # =======================================================

    def rebuild(

        self,

        cruise_speed: float | None = None,

        cruise_altitude: float | None = None,

    ):

        """
        Future Gain Scheduling.

        Rebuild A/B matrix
        according to flight condition.
        """

        if cruise_speed is not None:

            self.parameters.cruise_speed = cruise_speed

        if cruise_altitude is not None:

            self.parameters.cruise_altitude = cruise_altitude

        self.A = self._create_state_matrix()

        self.B = self._create_input_matrix()

    # =======================================================
    # Information
    # =======================================================

    def info(self):

        print("\n" + "=" * 60)

        print("Aircraft Information")

        print("=" * 60)

        print(f"Mass              : {self.parameters.mass:.1f} kg")
        print(f"Cruise Speed      : {self.parameters.cruise_speed:.1f} m/s")
        print(f"Cruise Altitude   : {self.parameters.cruise_altitude:.1f} m")

        print()

        print(
            "Open Loop Stable :",
            self.is_stable()
        )

        print(
            "Controllable     :",
            self.controllability_rank() == 6
        )

        print(
            "Observable       :",
            self.observability_rank() == 6
        )

        print()

        print("Eigenvalues")

        for value in self.eigenvalues():

            print(
                f"{value.real:10.5f}"
                f"{value.imag:+10.5f}j"
            )
    # =======================================================
    # Model Summary
    # =======================================================

    def summary(self):

        print("\nAircraft Model Summary")
        print("-" * 60)

        print("A Matrix")
        print(self.A)

        print("\nB Matrix")
        print(self.B)

        print("\nC Matrix")
        print(self.C)

        print("\nD Matrix")
        print(self.D)

        print("\nState Dimension :", self.A.shape[0])

        print("Control Dimension :", self.B.shape[1])

        print()

    # =======================================================
    # Utility
    # =======================================================

    @staticmethod
    def deg2rad(angle):

        return np.deg2rad(angle)

    @staticmethod
    def rad2deg(angle):

        return np.rad2deg(angle)

    # =======================================================
    # Reset
    # =======================================================

    def reset(self):

        return self.trim_state(), self.trim_control()

    # =======================================================
    # String
    # =======================================================

    def __repr__(self):

        return (

            "Aircraft("

            f"mass={self.parameters.mass}, "

            f"speed={self.parameters.cruise_speed}, "

            f"altitude={self.parameters.cruise_altitude}"

            ")"

        )


# ===========================================================
# Standalone Test
# ===========================================================

if __name__ == "__main__":

    aircraft = Aircraft()

    aircraft.validate()

    aircraft.info()

    aircraft.summary()

    x = np.array([

        np.deg2rad(5),

        0.0,

        np.deg2rad(3),

        0.0,

        np.deg2rad(2),

        0.0

    ])

    u = np.zeros(3)

    x_dot = aircraft.dynamics(

        x,

        u

    )

    print("\nState")

    print(x)

    print()

    print("Derivative")

    print(x_dot)

    print()

    print("Output")

    print(

        aircraft.output(

            x,

            u

        )

    )