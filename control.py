"""
control.py
-------------------------------------------------------
Aircraft control input representation.

Control Vector

u =

[aileron,
 elevator,
 rudder]

Units

rad
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class ControlInput:
    """
    Aircraft control surface commands.

    Parameters
    ----------
    aileron : float
        Aileron deflection (rad)

    elevator : float
        Elevator deflection (rad)

    rudder : float
        Rudder deflection (rad)
    """

    aileron: float = 0.0
    elevator: float = 0.0
    rudder: float = 0.0

    # --------------------------------------------------

    def to_vector(self) -> np.ndarray:
        """
        Convert to numpy vector.
        """

        return np.array([
            self.aileron,
            self.elevator,
            self.rudder
        ], dtype=float)

    # --------------------------------------------------

    @classmethod
    def from_vector(cls, vector: np.ndarray) -> "ControlInput":
        """
        Create control input from vector.
        """

        vector = np.asarray(vector).flatten()

        if vector.size != 3:
            raise ValueError(
                "Control vector must contain 3 elements."
            )

        return cls(
            aileron=vector[0],
            elevator=vector[1],
            rudder=vector[2]
        )

    # --------------------------------------------------

    def copy(self) -> "ControlInput":
        """
        Deep copy.
        """

        return ControlInput.from_vector(
            self.to_vector()
        )

    # --------------------------------------------------

    @staticmethod
    def zero() -> "ControlInput":
        """
        Zero control input.
        """

        return ControlInput()

    # --------------------------------------------------

    def clip(
        self,
        max_aileron: float,
        max_elevator: float,
        max_rudder: float
    ) -> "ControlInput":
        """
        Apply control surface saturation.
        """

        self.aileron = np.clip(
            self.aileron,
            -max_aileron,
            max_aileron
        )

        self.elevator = np.clip(
            self.elevator,
            -max_elevator,
            max_elevator
        )

        self.rudder = np.clip(
            self.rudder,
            -max_rudder,
            max_rudder
        )

        return self

    # --------------------------------------------------

    def as_degrees(self) -> dict[str, float]:
        """
        Return degrees.
        """

        return {

            "aileron": np.rad2deg(self.aileron),

            "elevator": np.rad2deg(self.elevator),

            "rudder": np.rad2deg(self.rudder)

        }

    # --------------------------------------------------

    def __str__(self):

        deg = self.as_degrees()

        return (
            f"Aileron : {deg['aileron']:8.3f} deg\n"
            f"Elevator: {deg['elevator']:8.3f} deg\n"
            f"Rudder  : {deg['rudder']:8.3f} deg"
        )