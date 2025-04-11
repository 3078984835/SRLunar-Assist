import zhplot
import matplotlib.pyplot as plt

class Plot:
    def __init__(self):
        pass

    def plot_robot(self, current_time, robot_data):
        fig, ax = plt.subplots(layout="constrained")
        l1, = ax.plot(current_time, robot_data[3, :], label="LJ2", linestyle="-", color="blue")
        l2, = ax.plot(current_time, robot_data[5, :], label="LJ3", linestyle="-", color="red")
        ax2 = ax.twinx()
        l3, = ax2.plot(current_time, robot_data[7, :], label="LP4", linestyle="-", color="green")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Degrees")
        ax.set_title("机器人左腿髋、膝关节角度及末端节位置")
        ax2.legend([ l1, l2, l3], ["LJ2", "LJ3", "LP4"])
        plt.show()