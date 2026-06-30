"""
============================================================
Aircraft Attitude Control Simulator

Main Program

Author : TaeWoo + ChatGPT
============================================================
"""

import numpy as np

from aircraft import Aircraft
from controller import LQRController
from simulator import Simulator
from visualization import Visualizer


def main():

    # =====================================================
    # Create Objects
    # =====================================================

    aircraft = Aircraft()

    controller = LQRController(

        aircraft

    )

    simulator = Simulator(

        aircraft=aircraft,

        controller=controller,

        dt=0.01,

        simulation_time=20.0

    )

    visualizer = Visualizer()

    # =====================================================
    # Initial State
    # =====================================================

    initial_state = np.array([

        np.deg2rad(10),

        0.0,

        np.deg2rad(5),

        0.0,

        np.deg2rad(2),

        0.0

    ])

    # =====================================================
    # Reference
    # =====================================================

    reference = np.zeros(6)

    # =====================================================
    # Simulation
    # =====================================================

    (

        time,

        states,

        controls,

        references,

        wind

    ) = simulator.run(

        initial_state,

        reference

    )

    # =====================================================
    # Aircraft Information
    # =====================================================

    aircraft.info()

    controller.summary()

    # =====================================================
    # Performance
    # =====================================================

    roll_rmse = visualizer.rmse(

        states[:,0],

        references[:,0]

    )

    pitch_rmse = visualizer.rmse(

        states[:,2],

        references[:,2]

    )

    yaw_rmse = visualizer.rmse(

        states[:,4],

        references[:,4]

    )

    print()

    print("="*60)

    print("Simulation Result")

    print("="*60)

    print()

    print(

        f"Roll RMSE  : {np.rad2deg(roll_rmse):.3f} deg"

    )

    print(

        f"Pitch RMSE : {np.rad2deg(pitch_rmse):.3f} deg"

    )

    print(

        f"Yaw RMSE   : {np.rad2deg(yaw_rmse):.3f} deg"

    )

    print()

    print(

        "Final State (deg)"

    )

    print(

        np.rad2deg(

            states[-1]

        )

    )

    print()

    print(

        "Final Control (deg)"

    )

    print(

        np.rad2deg(

            controls[-1]

        )

    )

    # =====================================================
    # Save CSV
    # =====================================================

    visualizer.export_csv(

        "simulation_result.csv",

        time,

        states,

        controls

    )

    # =====================================================
    # Plot
    # =====================================================

    visualizer.plot_attitude(

        time,

        states,

        references

    )

    visualizer.plot_rates(

        time,

        states

    )

    visualizer.plot_control(

        time,

        controls

    )

    visualizer.show()


if __name__ == "__main__":

    main()