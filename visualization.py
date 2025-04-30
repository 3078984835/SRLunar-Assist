"""
可视化模块 - 用于绘制仿真数据图表
"""

import zhplot
import matplotlib.pyplot as plt


class Visualization:
    """数据可视化类"""

    def __init__(self):
        """初始化可视化类"""
        pass

    def plot_robot_walk(self, current_time, robot_data):
        """绘制机器人步行数据图表

        Args:
            current_time: 时间数据列表
            robot_data: 机器人运动数据矩阵
        """
        fig, ax = plt.subplots(layout="constrained")

        # 绘制左腿关节角度
        (l1,) = ax.plot(
            current_time, robot_data[2, :], label="LJ2", linestyle="-", color="blue"
        )
        (l2,) = ax.plot(
            current_time, robot_data[4, :], label="LJ3", linestyle="-", color="red"
        )

        # 创建双y轴，用于显示位置数据
        ax2 = ax.twinx()
        (l3,) = ax2.plot(
            current_time, robot_data[6, :], label="LP4", linestyle="-", color="green"
        )

        # 设置图表标签
        ax.set_xlabel("时间 (秒)")
        ax.set_ylabel("角度 (度)")
        ax2.set_ylabel("位置 (米)")
        ax.set_title("机器人左腿髋、膝关节角度及末端节位置")

        # 添加图例
        ax2.legend([l1, l2, l3], ["髋关节", "膝关节", "末端位置"])

        plt.show()

    def plot_bill_position(self, current_time, bill_position):
        """绘制Bill位置数据图表

        Args:
            current_time: 时间数据列表
            bill_position: Bill位置数据矩阵
        """
        fig, ax = plt.subplots(layout="constrained")

        # 绘制X、Y坐标变化
        (l1,) = ax.plot(
            current_time,
            bill_position[0, :],
            label="X坐标",
            linestyle="-",
            color="blue",
        )
        (l2,) = ax.plot(
            current_time, bill_position[1, :], label="Y坐标", linestyle="-", color="red"
        )

        # 设置图表标签
        ax.set_xlabel("时间 (秒)")
        ax.set_ylabel("位置 (米)")
        ax.set_title("Bill位置变化")

        # 添加图例
        ax.legend([l1, l2], ["X坐标", "Y坐标"])

        plt.show()

    def plot_robot_prone(self, current_time, robot_data):
        """绘制机器人趴姿到站姿数据图表

        Args:
            current_time: 时间数据列表
            robot_data: 机器人运动数据矩阵
        """
        fig, ax = plt.subplots(layout="constrained")

        # 绘制方向和关节角度
        (l0,) = ax.plot(
            current_time, robot_data[0, :], label="θ0", linestyle="-", color="orange"
        )
        (l1,) = ax.plot(
            current_time, robot_data[1, :], label="J2", linestyle="-", color="blue"
        )
        (l2,) = ax.plot(
            current_time, robot_data[2, :], label="J3", linestyle="-", color="red"
        )

        # 创建双y轴，用于显示位置数据
        ax2 = ax.twinx()
        (l3,) = ax2.plot(
            current_time, robot_data[3, :], label="P4", linestyle="-", color="green"
        )

        # 设置图表标签
        ax.set_xlabel("时间 (秒)")
        ax.set_ylabel("角度 (度)")
        ax2.set_ylabel("位置 (米)")
        ax.set_title("机器人髋、膝关节角度及末端节位置")

        # 添加图例
        ax2.legend([l0, l1, l2, l3], ["θ0", "J2", "J3", "P4"])

        plt.show()
