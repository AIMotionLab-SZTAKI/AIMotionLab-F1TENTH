
import matplotlib.pyplot as plt

from typing import Union

import matplotlib as mpl

import os

import pickle

import numpy as np

# Global variable to store clicked points
clicked_points = []


from aimotion_f1tenth_utils.Trajectory import Trajectory

mpl.rcParams["text.usetex"] = False ##Because of the import


def onclick(event):
    # Check if the click event occurred within the plot area
    if event.inaxes is not None:
        # Append the clicked point to the global list
        clicked_points.append([event.xdata, event.ydata])
        # Display the clicked points
        print(f"Clicked points: {clicked_points}")

        # Plot the clicked points
        plt.scatter(*zip(*clicked_points), color='red')
        plt.draw()



def main():


    # Create a figure and axis
    fig, ax = plt.subplots(figsize=(5, 5))

    # Set up the click event handler
    cid = fig.canvas.mpl_connect('button_press_event', onclick)


    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])


    print("Close window to evalute the spline")

    # Display the plot
    plt.show()

    if len(clicked_points) < 2:
        print("Select at least two points")
        return
    
    input_data = float(input("Enter speed (m/s): "))

    while type(input_data) != float and type(input_data) != int:
        input_data = float(input("Enter speed (m/s): "))

    speed = input_data

    if speed > 2.5:
        speed = 2.5
    if speed < -2.5:
        speed = -2.5

    
    name = input("Enter trajectory name: ")

    traj1 = Trajectory(name)
    traj1.build_from_points_const_speed(np.array(clicked_points), 0.01, 3, speed)

    path_data = {"pos_tck": traj1.pos_tck,
            "evol_tck": traj1.evol_tck,
            "reversed": traj1.reversed}
    

    traj_id = name

    with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), traj_id + ".traj"), "wb") as file:
        pickle.dump(obj=path_data, file= file)
    


if __name__ == "__main__":
    main()