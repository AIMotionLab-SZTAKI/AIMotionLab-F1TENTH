from aimotion_f1tenth_utils.F1Client import F1Client
import matplotlib.pyplot as plt
import numpy as np
import time


class DynamicSubplots:
    def __init__(self):
        self.fig, self.axes = plt.subplots(3, 2)
        self.lines = [ax.plot([], [])[0] for ax in self.axes.flat]
        self.axes[0][0].set_title("x")
        self.axes[0][0].set_ylabel("x")
        self.axes[0][0].set_xlabel("t")


        self.axes[1][0].set_title("y")
        self.axes[1][0].set_ylabel("y")
        self.axes[1][0].set_xlabel("t")


        self.axes[2][0].set_title("phi")
        self.axes[2][0].set_ylabel("phi")
        self.axes[2][0].set_xlabel("t")


        self.axes[0][1].set_title("v_xi")
        self.axes[0][1].set_ylabel("v_xi")
        self.axes[0][1].set_xlabel("t")

        self.axes[1][1].set_title("v_eta")
        self.axes[1][1].set_ylabel("v_eta")
        self.axes[1][1].set_xlabel("t")


        self.axes[2][1].set_title("omega")
        self.axes[2][1].set_ylabel("omega")
        self.axes[2][1].set_xlabel("t")


        
        self.data = [([], []) for _ in range(6)]
        plt.ion()

    def add_point(self, subplot_idx, x, y):
        if 0 <= subplot_idx < 6:
            self.data[subplot_idx][0].append(x)
            self.data[subplot_idx][1].append(y)
            self._update_plot(subplot_idx)

    def _update_plot(self, subplot_idx):
        line = self.lines[subplot_idx]
        xdata, ydata = self.data[subplot_idx]
        line.set_data(xdata, ydata)
        ax = self.axes.flat[subplot_idx]
        ax.relim()
        ax.autoscale_view()
    def full_update(self):
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
 

    def reset_plots(self):
        self.data = [([], []) for _ in range(6)]
        for line in self.lines:
            line.set_data([], [])
        for ax in self.axes.flat:
            ax.relim()
            ax.autoscale_view()
        self.fig.canvas.draw()

    def show(self):
        plt.tight_layout()
        plt.ion()
        plt.show()

def main():
    plotter  = DynamicSubplots()
    plotter.show()
    car = F1Client("JoeBush1")
    while True:
        state = car.get_state()
        t = time.time()
        #time.sleep(1/60)
        print(state)
        try:
            plotter.add_point(0, t, state[0])
            plotter.add_point(2, t, state[1])
            plotter.add_point(4, t, state[2])
            plotter.add_point(1, t, state[3])
            plotter.add_point(3, t, state[4])
            plotter.add_point(5, t, state[5])
            for plot_index in range(6):
                plotter._update_plot(plot_index)
            plotter.full_update()
        except Exception as e:
            print(e)



if __name__ == "__main__":
    main()