"""
simulator.py
------------------------------------------------------------
Aircraft Simulation Engine

- RK4 Integrator
- Wind Disturbance
- Sensor Noise
- Data Logger

Author : TaeWoo + ChatGPT
"""

from __future__ import annotations

import numpy as np

from aircraft import Aircraft
from controller import LQRController


class Simulator:

    def __init__(
        self,
        aircraft: Aircraft,
        controller: LQRController,
        dt: float = 0.01,
        simulation_time: float = 20.0,
    ):

        self.aircraft = aircraft
        self.controller = controller

        self.dt = dt
        self.simulation_time = simulation_time

        self.steps = int(
            simulation_time / dt
        )

        self.reset()

    # =====================================================
    # Reset
    # =====================================================

    def reset(self):

        self.time = []

        self.state_history = []

        self.control_history = []

        self.reference_history = []

        self.wind_history = []

    # =====================================================
    # Wind Model
    # =====================================================

    def wind(self):

        """
        Simple Gaussian disturbance.
        """

        sigma = 0.002

        return np.random.normal(

            0.0,

            sigma,

            6

        )

    # =====================================================
    # Sensor Noise
    # =====================================================

    def sensor_noise(self):

        """
        Simulated IMU noise.
        """

        sigma = 0.0005

        return np.random.normal(

            0.0,

            sigma,

            6

        )

    # =====================================================
    # RK4
    # =====================================================

    def rk4_step(

        self,

        state,

        control,

        disturbance

    ):

        dt = self.dt

        f = self.aircraft.dynamics

        k1 = f(

            state,

            control,

            disturbance

        )

        k2 = f(

            state + 0.5 * dt * k1,

            control,

            disturbance

        )

        k3 = f(

            state + 0.5 * dt * k2,

            control,

            disturbance

        )

        k4 = f(

            state + dt * k3,

            control,

            disturbance

        )

        return state + (

            dt / 6.0

        ) * (

            k1

            + 2 * k2

            + 2 * k3

            + k4

        )

    # =====================================================
    # Logger
    # =====================================================

    def log(

        self,

        t,

        state,

        control,

        reference,

        wind

    ):

        self.time.append(t)

        self.state_history.append(

            state.copy()

        )

        self.control_history.append(

            control.copy()

        )

        self.reference_history.append(

            reference.copy()

        )

        self.wind_history.append(

            wind.copy()

        )
    # =====================================================
    # Simulation Loop
    # =====================================================

    def run(
        self,
        initial_state: np.ndarray,
        reference: np.ndarray,
    ):
        """
        Execute simulation.
        """

        self.reset()

        state = np.asarray(
            initial_state,
            dtype=float
        ).reshape(6)

        reference = np.asarray(
            reference,
            dtype=float
        ).reshape(6)

        self.controller.reset()

        for step in range(self.steps):

            t = step * self.dt

            # ---------------------------------------------
            # Sensor Measurement
            # ---------------------------------------------

            measured_state = (
                state
                + self.sensor_noise()
            )

            # ---------------------------------------------
            # LQR Control
            # ---------------------------------------------

            control = self.controller.control(
                measured_state,
                reference,
                self.dt
            )

            control_vector = control.to_vector()

            # ---------------------------------------------
            # Environment
            # ---------------------------------------------

            disturbance = self.wind()

            # ---------------------------------------------
            # Aircraft Dynamics
            # ---------------------------------------------

            state = self.rk4_step(
                state,
                control_vector,
                disturbance
            )

            # ---------------------------------------------
            # Logging
            # ---------------------------------------------

            self.log(
                t=t,
                state=state,
                control=control_vector,
                reference=reference,
                wind=disturbance
            )

        return (
            np.asarray(self.time),
            np.asarray(self.state_history),
            np.asarray(self.control_history),
            np.asarray(self.reference_history),
            np.asarray(self.wind_history),
        )

    # =====================================================
    # Final State
    # =====================================================

    @property
    def final_state(self):

        if not self.state_history:

            return None

        return self.state_history[-1]

    # =====================================================
    # Final Control
    # =====================================================

    @property
    def final_control(self):

        if not self.control_history:

            return None

        return self.control_history[-1]

    # =====================================================
    # Number of Samples
    # =====================================================

    @property
    def sample_count(self):

        return len(self.time)
# ==========================================================
# Standalone Test
# ==========================================================

if __name__ == "__main__":

    aircraft = Aircraft()

    controller = LQRController(
        aircraft
    )

    simulator = Simulator(

        aircraft,

        controller,

        dt=0.01,

        simulation_time=20.0

    )

    initial_state = np.array([

        np.deg2rad(8),

        0.0,

        np.deg2rad(5),

        0.0,

        np.deg2rad(3),

        0.0

    ])

    reference = np.zeros(6)

    (
        time,

        states,

        controls,

        references,

        wind,

    ) = simulator.run(

        initial_state,

        reference

    )

    print()

    print("=" * 60)

    print("Simulation Complete")

    print("=" * 60)

    print()

    print("Samples")

    print(simulator.sample_count)

    print()

    print("Final State (deg)")

    print(np.rad2deg(simulator.final_state))

    print()

    print("Final Control (deg)")

    print(np.rad2deg(simulator.final_control))