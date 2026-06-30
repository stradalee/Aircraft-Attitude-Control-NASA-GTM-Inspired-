"""
state.py
-------------------------------------------------------
Aircraft attitude state representation.

State Vector

x =

[phi,
 p,
 theta,
 q,
 psi,
 r]

Units

phi, theta, psi : rad
p, q, r          : rad/s
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class AircraftState:
    """
    Aircraft attitude state.

    Attributes
    ----------
    roll : float
        Roll angle (rad)

    roll_rate : float
        Roll rate (rad/s)

    pitch : float
        Pitch angle (rad)

    pitch_rate : float
        Pitch rate (rad/s)

    yaw : float
        Yaw angle (rad)

    yaw_rate : float
        Yaw rate (rad/s)
    """

    roll: float = 0.0
    roll_rate: float = 0.0

    pitch: float = 0.0
    pitch_rate: float = 0.0

    yaw: float = 0.0
    yaw_rate: float = 0.0

    # --------------------------------------------------

    def to_vector(self) -> np.ndarray:
        """
        Convert state to column vector.
        """

        return np.array([
            self.roll,
            self.roll_rate,
            self.pitch,
            self.pitch_rate,
            self.yaw,
            self.yaw_rate
        ], dtype=float)

    # --------------------------------------------------

    @classmethod
    def from_vector(cls, vector: np.ndarray) -> "AircraftState":
        """
        Create AircraftState from vector.
        """

        vector = np.asarray(vector).flatten()

        if vector.size != 6:
            raise ValueError(
                "Aircraft state vector must contain 6 elements."
            )

        return cls(
            roll=vector[0],
            roll_rate=vector[1],
            pitch=vector[2],
            pitch_rate=vector[3],
            yaw=vector[4],
            yaw_rate=vector[5]
        )

    # --------------------------------------------------

    def copy(self) -> "AircraftState":
        """
        Return a deep copy of the state.
        """

        return AircraftState.from_vector(self.to_vector())

    # --------------------------------------------------

    def as_degrees(self) -> dict[str, float]:
        """
        Return state as degrees.
        """

        return {

            "roll": np.rad2deg(self.roll),
            "roll_rate": np.rad2deg(self.roll_rate),

            "pitch": np.rad2deg(self.pitch),
            "pitch_rate": np.rad2deg(self.pitch_rate),

            "yaw": np.rad2deg(self.yaw),
            "yaw_rate": np.rad2deg(self.yaw_rate)

        }

    # --------------------------------------------------

    @staticmethod
    def zero() -> "AircraftState":
        """
        Zero state.
        """

        return AircraftState()

    # --------------------------------------------------

    def __str__(self) -> str:

        deg = self.as_degrees()

        return (
            f"Roll  : {deg['roll']:8.3f} deg\n"
            f"Pitch : {deg['pitch']:8.3f} deg\n"
            f"Yaw   : {deg['yaw']:8.3f} deg\n"
            f"P     : {deg['roll_rate']:8.3f} deg/s\n"
            f"Q     : {deg['pitch_rate']:8.3f} deg/s\n"
            f"R     : {deg['yaw_rate']:8.3f} deg/s"
        )