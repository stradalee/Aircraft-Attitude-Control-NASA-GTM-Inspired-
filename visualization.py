"""
visualization.py
------------------------------------------------------------
Visualization & Performance Analysis

Author : TaeWoo + ChatGPT
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


class Visualizer:

    def __init__(self):

        plt.style.use("ggplot")

    # =====================================================
    # RMSE
    # =====================================================

    @staticmethod
    def rmse(actual, reference):

        actual = np.asarray(actual)

        reference = np.asarray(reference)

        return np.sqrt(

            np.mean(

                (actual-reference)**2

            )

        )

    # =====================================================
    # Peak Overshoot
    # =====================================================

    @staticmethod
    def overshoot(response, reference=0.0):

        response = np.asarray(response)

        peak = np.max(np.abs(response))

        if np.abs(reference) < 1e-8:

            return peak

        return (

            np.abs(peak-reference)

            /

            np.abs(reference)

        ) * 100.0

    # =====================================================
    # Settling Time
    # =====================================================

    @staticmethod
    def settling_time(

        time,

        response,

        reference=0.0,

        tolerance=0.02

    ):

        response = np.asarray(response)

        time = np.asarray(time)

        limit = tolerance

        if abs(reference) > 1e-8:

            limit *= abs(reference)

        for i in range(len(response)):

            if np.all(

                np.abs(

                    response[i:]-reference

                ) <= limit

            ):

                return time[i]

        return np.nan

    # =====================================================
    # Rise Time
    # =====================================================

    @staticmethod
    def rise_time(

        time,

        response,

        reference

    ):

        response = np.asarray(response)

        time = np.asarray(time)

        start = 0.1*reference

        end = 0.9*reference

        t1 = None

        t2 = None

        for i,v in enumerate(response):

            if t1 is None and v>=start:

                t1=time[i]

            if t2 is None and v>=end:

                t2=time[i]

        if t1 is None or t2 is None:

            return np.nan

        return t2-t1
    
    # =====================================================
    # Plot Attitude
    # =====================================================

    def plot_attitude(

        self,

        time,

        states,

        references

    ):

        plt.figure(figsize=(12,8))

        labels=[

            "Roll",

            "Pitch",

            "Yaw"

        ]

        index=[0,2,4]

        for i,j in enumerate(index):

            plt.subplot(3,1,i+1)

            plt.plot(

                time,

                np.rad2deg(states[:,j]),

                label="Response"

            )

            plt.plot(

                time,

                np.rad2deg(references[:,j]),

                "--",

                label="Reference"

            )

            plt.ylabel(labels[i]+" (deg)")

            plt.grid(True)

            plt.legend()

        plt.xlabel("Time (s)")

        plt.tight_layout()

    # =====================================================
    # Plot Angular Rate
    # =====================================================

    def plot_rates(

        self,

        time,

        states

    ):

        plt.figure(figsize=(12,8))

        labels=[

            "P",

            "Q",

            "R"

        ]

        index=[1,3,5]

        for i,j in enumerate(index):

            plt.subplot(3,1,i+1)

            plt.plot(

                time,

                np.rad2deg(states[:,j])

            )

            plt.ylabel(

                labels[i]+" (deg/s)"

            )

            plt.grid(True)

        plt.xlabel("Time (s)")

        plt.tight_layout()
    # =====================================================
    # Plot Control
    # =====================================================

    def plot_control(

        self,

        time,

        controls

    ):

        plt.figure(figsize=(12,8))

        labels=[

            "Aileron",

            "Elevator",

            "Rudder"

        ]

        for i in range(3):

            plt.subplot(3,1,i+1)

            plt.plot(

                time,

                np.rad2deg(controls[:,i])

            )

            plt.ylabel(

                labels[i]+" (deg)"

            )

            plt.grid(True)

        plt.xlabel("Time (s)")

        plt.tight_layout()

    # =====================================================
    # CSV Export
    # =====================================================

    def export_csv(

        self,

        filename,

        time,

        states,

        controls

    ):

        df=pd.DataFrame({

            "Time":time,

            "Roll":np.rad2deg(states[:,0]),

            "RollRate":np.rad2deg(states[:,1]),

            "Pitch":np.rad2deg(states[:,2]),

            "PitchRate":np.rad2deg(states[:,3]),

            "Yaw":np.rad2deg(states[:,4]),

            "YawRate":np.rad2deg(states[:,5]),

            "Aileron":np.rad2deg(controls[:,0]),

            "Elevator":np.rad2deg(controls[:,1]),

            "Rudder":np.rad2deg(controls[:,2])

        })

        df.to_csv(

            filename,

            index=False

        )

    # =====================================================
    # Show
    # =====================================================

    @staticmethod
    def show():

        plt.show()