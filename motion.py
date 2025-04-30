"""
运动控制模块 - 实现各种机器人运动模式
"""

import math
import numpy as np
from constants import (
    LEG_WAYPOINTS,
    KNEE_WAYPOINTS,
    ANKLE_WAYPOINTS,
    SHOULDER_WAYPOINTS,
    ELBOW_WAYPOINTS,
    AMPLITUDE_COEFFICIENT,
    JOINT2_LENGTH,
    JOINT3_LENGTH,
    JOINT4_LENGTH,
    HEIGHT,
    BILL_VELOCITY,
    SIT_TO_STAND_WAYPOINTS,
    PRONE_TO_STAND_WAYPOINTS,
    FALL_TO_STAND_WAYPOINTS,
)


class Motion:
    """机器人运动控制类"""

    def __init__(self, api):
        """初始化运动控制器

        Args:
            api: SimulationAPI实例
        """
        self.api = api

        # 数据记录初始化
        self.current_time = []
        self.robot_data = np.zeros((8, 0))
        self.bill_data = np.zeros((13, 0))
        self.bill_position_data = np.zeros((3, 0))
        self.test_data = np.zeros((4, 0))

        # Bill的位置和方向
        self.bill_position = [0, 0]
        self.bill_orientation = 0
        self.bill_velocity = BILL_VELOCITY

        # 用于追踪仿真状态的变量
        self.dist = 0
        self.st = 0
        self.st_initialized = False

        # 加载路径数据
        self._load_path_data()

    def _load_path_data(self):
        """加载并处理路径数据"""
        # 读取路径数据
        path_data = self.api.unpack_double_table(
            self.api.read_custom_buffer_data(self.api.path_handle, "PATH")
        )

        # 将数据转换为7列的矩阵（位置+四元数）
        self.path_matrix = np.array(path_data).reshape(-1, 7)

        # 获取路径的位姿
        self.path_pose = self.api.get_object_pose(self.api.path_handle)

        # 提取位置数据并转换为列表
        self.path_positions = self.path_matrix[:, :3].tolist()

        # 扁平化为一维列表
        self.path_positions = [
            item for sublist in self.path_positions for item in sublist
        ]

        # 将路径坐标转换为世界坐标系
        self.path_positions = self.api.multiply_vector(
            self.path_pose, self.path_positions
        )

        # 计算路径总长度
        self.path_lengths, self.path_total_length = self.api.get_path_lengths(
            self.path_positions, 3
        )

    def interpolate_angle(self, waypoints, t):
        """线性插值计算关节角度

        Args:
            waypoints: 关键帧角度列表
            t: 归一化时间 (0<=t<=1)

        Returns:
            float: 插值后的角度值
        """
        total_frames = len(waypoints)
        segment_count = total_frames - 1
        pos = t * segment_count
        i = int(math.floor(pos))

        if i >= segment_count:
            return waypoints[-1]

        fraction = pos - i
        angle = waypoints[i] + (waypoints[i + 1] - waypoints[i]) * fraction
        return angle

    def move_robot(self, dist):
        """控制机器人外肢体运动

        Args:
            dist: 归一化距离值 (0<=dist<=1)
        """
        # 计算左右腿相位
        t1 = dist % 1.0
        t2 = (t1 + 0.5) % 1.0

        # 插值计算关节角度
        right_leg_angle = self.interpolate_angle(LEG_WAYPOINTS, t1)
        right_knee_angle = self.interpolate_angle(KNEE_WAYPOINTS, t1)
        left_leg_angle = self.interpolate_angle(LEG_WAYPOINTS, t2)
        left_knee_angle = self.interpolate_angle(KNEE_WAYPOINTS, t2)

        # 根据增幅系数调整角度
        right_leg_angle_adj = right_leg_angle * (1 + AMPLITUDE_COEFFICIENT)
        right_knee_angle_adj = right_knee_angle * (1 + AMPLITUDE_COEFFICIENT)
        left_leg_angle_adj = left_leg_angle * (1 + AMPLITUDE_COEFFICIENT)
        left_knee_angle_adj = left_knee_angle * (1 + AMPLITUDE_COEFFICIENT)

        # 设置机器人关节角度
        self.api.set_joint_position(self.api.right_j2_handle, right_leg_angle_adj)
        self.api.set_joint_position(self.api.right_j3_handle, right_knee_angle_adj)
        self.api.set_joint_position(self.api.left_j2_handle, left_leg_angle_adj)
        self.api.set_joint_position(self.api.left_j3_handle, left_knee_angle_adj)

        # 计算末端节位置（不同角度范围计算）
        right_length = (
            (HEIGHT - JOINT2_LENGTH * math.cos(abs(right_leg_angle_adj)))
            / (math.cos(abs(right_leg_angle_adj) - abs(right_knee_angle_adj)))
            - JOINT3_LENGTH
            - JOINT4_LENGTH
        )
        if right_length > 0:
            right_length = 0

        left_length = (
            (HEIGHT - JOINT2_LENGTH * math.cos(abs(left_leg_angle_adj)))
            / (math.cos(abs(left_leg_angle_adj) - abs(left_knee_angle_adj)))
            - JOINT3_LENGTH
            - JOINT4_LENGTH
        )
        if left_length > 0:
            left_length = 0

        left_length_alt = (
            (HEIGHT - JOINT2_LENGTH * math.cos(abs(right_leg_angle_adj)))
            / (math.cos(abs(right_leg_angle_adj) + abs(right_knee_angle_adj)))
            - JOINT3_LENGTH
            - JOINT4_LENGTH
        )
        if left_length_alt > 0:
            left_length_alt = 0

        right_length_alt = (
            (HEIGHT - JOINT2_LENGTH * math.cos(abs(left_leg_angle_adj)))
            / (math.cos(abs(left_leg_angle_adj) + abs(left_knee_angle_adj)))
            - JOINT3_LENGTH
            - JOINT4_LENGTH
        )
        if right_length_alt > 0:
            right_length_alt = 0

        # 获取当前位置
        current_right_p4_pos = self.api.get_joint_position(self.api.right_p4_handle)
        current_left_p4_pos = self.api.get_joint_position(self.api.left_p4_handle)

        # 步进速率（平滑过渡）
        step_rate = 0.001  # 落地时的延展速率
        retract_rate = 0.001  # 收回速率

        # 设置末端节位置，使用平滑过渡
        if t1 < 0.5:
            # 左腿前进，右腿后退
            # 右腿处理 - 抬起阶段
            if right_leg_angle_adj < -0.1:  # 当右腿抬起时
                target_right_p4 = -0.3  # 收回目标位置
                if current_right_p4_pos > target_right_p4:
                    new_right_p4_pos = max(
                        current_right_p4_pos - retract_rate, target_right_p4
                    )
                    self.api.set_joint_position(
                        self.api.right_p4_handle, new_right_p4_pos
                    )

            # 左腿处理 - 着地阶段
            if left_leg_angle_adj > 0:  # 左腿前伸
                target_left_p4 = right_length_alt  # 计算左腿目标位置
                # 平滑过渡到目标位置
                if abs(current_left_p4_pos - target_left_p4) > step_rate:
                    if current_left_p4_pos < target_left_p4:
                        new_left_p4_pos = min(
                            current_left_p4_pos + step_rate, target_left_p4
                        )
                    else:
                        new_left_p4_pos = max(
                            current_left_p4_pos - step_rate, target_left_p4
                        )
                    self.api.set_joint_position(
                        self.api.left_p4_handle, new_left_p4_pos
                    )
                else:
                    self.api.set_joint_position(self.api.left_p4_handle, target_left_p4)
            else:  # 左腿后撤
                target_left_p4 = left_length
                # 平滑过渡到目标位置
                if abs(current_left_p4_pos - target_left_p4) > step_rate:
                    if current_left_p4_pos < target_left_p4:
                        new_left_p4_pos = min(
                            current_left_p4_pos + step_rate, target_left_p4
                        )
                    else:
                        new_left_p4_pos = max(
                            current_left_p4_pos - step_rate, target_left_p4
                        )
                    self.api.set_joint_position(
                        self.api.left_p4_handle, new_left_p4_pos
                    )
                else:
                    self.api.set_joint_position(self.api.left_p4_handle, target_left_p4)
        else:
            # 左腿后退，右腿前进
            # 左腿处理 - 抬起阶段
            if left_leg_angle_adj < -0.1:  # 当左腿抬起时
                target_left_p4 = -0.3  # 收回目标位置
                if current_left_p4_pos > target_left_p4:
                    new_left_p4_pos = max(
                        current_left_p4_pos - retract_rate, target_left_p4
                    )
                    self.api.set_joint_position(
                        self.api.left_p4_handle, new_left_p4_pos
                    )

            # 右腿处理 - 着地阶段
            if right_leg_angle_adj > 0:  # 右腿前伸
                target_right_p4 = left_length_alt  # 计算右腿目标位置
                # 平滑过渡到目标位置
                if abs(current_right_p4_pos - target_right_p4) > step_rate:
                    if current_right_p4_pos < target_right_p4:
                        new_right_p4_pos = min(
                            current_right_p4_pos + step_rate, target_right_p4
                        )
                    else:
                        new_right_p4_pos = max(
                            current_right_p4_pos - step_rate, target_right_p4
                        )
                    self.api.set_joint_position(
                        self.api.right_p4_handle, new_right_p4_pos
                    )
                else:
                    self.api.set_joint_position(
                        self.api.right_p4_handle, target_right_p4
                    )
            else:  # 右腿后撤
                target_right_p4 = right_length
                # 平滑过渡到目标位置
                if abs(current_right_p4_pos - target_right_p4) > step_rate:
                    if current_right_p4_pos < target_right_p4:
                        new_right_p4_pos = min(
                            current_right_p4_pos + step_rate, target_right_p4
                        )
                    else:
                        new_right_p4_pos = max(
                            current_right_p4_pos - step_rate, target_right_p4
                        )
                    self.api.set_joint_position(
                        self.api.right_p4_handle, new_right_p4_pos
                    )
                else:
                    self.api.set_joint_position(
                        self.api.right_p4_handle, target_right_p4
                    )

        # 记录机器人关节数据
        robot_data_point = np.array(
            [
                0,
                0,
                math.degrees(self.api.get_joint_position(self.api.right_j2_handle)),
                math.degrees(self.api.get_joint_position(self.api.left_j2_handle)),
                math.degrees(self.api.get_joint_position(self.api.right_j3_handle)),
                math.degrees(self.api.get_joint_position(self.api.left_j3_handle)),
                self.api.get_joint_position(self.api.left_p4_handle),
                self.api.get_joint_position(self.api.right_p4_handle),
            ]
        ).reshape(8, 1)
        self.robot_data = np.hstack((self.robot_data, robot_data_point))

        # 推进仿真一步
        self.api.step()

    def move_bill(self, dist):
        """控制Bill人形角色的走路动作

        Args:
            dist: 归一化距离值 (0<=dist<=1)
        """
        t1 = dist % 1.0
        t2 = (t1 + 0.5) % 1.0

        # 计算并设置腿部关节角度
        left_leg_angle = self.interpolate_angle(LEG_WAYPOINTS, t1)
        left_knee_angle = self.interpolate_angle(KNEE_WAYPOINTS, t1)
        right_leg_angle = self.interpolate_angle(LEG_WAYPOINTS, t2)
        right_knee_angle = self.interpolate_angle(KNEE_WAYPOINTS, t2)

        self.api.set_joint_position(self.api.leg_joint_handles[0], left_leg_angle)
        self.api.set_joint_position(self.api.knee_joint_handles[0], left_knee_angle)
        self.api.set_joint_position(self.api.leg_joint_handles[1], right_leg_angle)
        self.api.set_joint_position(self.api.knee_joint_handles[1], right_knee_angle)

        # 计算并设置踝关节角度
        left_ankle_angle = self.interpolate_angle(ANKLE_WAYPOINTS, t1)
        right_ankle_angle = self.interpolate_angle(ANKLE_WAYPOINTS, t2)
        self.api.set_joint_position(self.api.ankle_joint_handles[0], left_ankle_angle)
        self.api.set_joint_position(self.api.ankle_joint_handles[1], right_ankle_angle)

        # 计算并设置上肢关节角度
        left_shoulder_angle = self.interpolate_angle(SHOULDER_WAYPOINTS, t1)
        right_shoulder_angle = self.interpolate_angle(SHOULDER_WAYPOINTS, t2)
        self.api.set_joint_position(
            self.api.shoulder_joint_handles[0], left_shoulder_angle
        )
        self.api.set_joint_position(
            self.api.shoulder_joint_handles[1], right_shoulder_angle
        )

        left_elbow_angle = self.interpolate_angle(ELBOW_WAYPOINTS, t1)
        right_elbow_angle = self.interpolate_angle(ELBOW_WAYPOINTS, t2)
        self.api.set_joint_position(self.api.elbow_joint_handles[0], left_elbow_angle)
        self.api.set_joint_position(self.api.elbow_joint_handles[1], right_elbow_angle)

        # 记录Bill关节角度数据
        bill_data_point = np.array(
            [
                0,
                0,
                math.degrees(
                    self.api.get_joint_position(self.api.leg_joint_handles[0])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.leg_joint_handles[1])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.knee_joint_handles[0])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.knee_joint_handles[1])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.ankle_joint_handles[0])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.ankle_joint_handles[1])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.shoulder_joint_handles[0])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.shoulder_joint_handles[1])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.elbow_joint_handles[0])
                ),
                math.degrees(
                    self.api.get_joint_position(self.api.elbow_joint_handles[1])
                ),
                math.degrees(self.api.get_joint_position(self.api.neck_joint_handle)),
            ]
        ).reshape(13, 1)
        self.bill_data = np.hstack((self.bill_data, bill_data_point))

    def displace_bill(self, step_size):
        """移动Bill沿路径前进

        Args:
            step_size: 移动步长
        """
        # 记录Bill位置数据
        position = np.array(self.api.get_object_position(self.api.bill_handle))
        self.bill_position_data = np.hstack(
            (self.bill_position_data, position.reshape(3, 1))
        )

        # 初始化Bill数据和起始时间（仅在第一次调用时）
        if not self.st_initialized:
            self.dist = 0
            self.st = self.api.get_simulation_time()
            self.st_initialized = True

        # 计算时间增量
        dt = self.api.get_simulation_time() - self.st
        self.st = self.api.get_simulation_time()

        # 计算移动距离
        self.dist += self.bill_velocity * dt * step_size

        # 计算当前位置和方向
        p = self.api.get_path_interpolated_config(
            self.path_positions, self.path_lengths, self.dist
        )
        current_position = self.api.get_object_position(self.api.bill_handle)
        dx = p[1] - current_position[1]
        dy = p[2] - current_position[2]

        # 更新Bill位置和方向
        self.api.set_object_orientation(
            self.api.bill_handle, [0, 0, math.atan2(dy, dx)]
        )
        self.api.set_object_position(self.api.bill_handle, p)

    def move_sit_to_stand(self, dist):
        """实现从坐姿到站姿的过渡

        Args:
            dist: 归一化距离值 (0<=dist<=1)
        """
        # 计算当前的时间比例
        t = dist % 1.0

        # 使用插值函数计算各关节角度
        shoulder_angle = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["shoulder"], t)
        elbow_angle = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["elbow"], t)
        leg_angle = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["leg"], t)
        knee_angle = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["knee"], t)
        ankle_angle = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["ankle"], t)
        neck_angle = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["neck"], t)
        j2_angle = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["J2"], t)
        j3_angle = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["J3"], t)
        p4_position = self.interpolate_angle(SIT_TO_STAND_WAYPOINTS["P4"], t)

        # 更新各关节位置
        self.api.set_joint_position(self.api.shoulder_joint_handles[0], shoulder_angle)
        self.api.set_joint_position(self.api.shoulder_joint_handles[1], shoulder_angle)
        self.api.set_joint_position(self.api.elbow_joint_handles[0], elbow_angle)
        self.api.set_joint_position(self.api.elbow_joint_handles[1], elbow_angle)
        self.api.set_joint_position(self.api.leg_joint_handles[0], leg_angle)
        self.api.set_joint_position(self.api.leg_joint_handles[1], leg_angle)
        self.api.set_joint_position(self.api.knee_joint_handles[0], knee_angle)
        self.api.set_joint_position(self.api.knee_joint_handles[1], knee_angle)
        self.api.set_joint_position(self.api.ankle_joint_handles[0], ankle_angle)
        self.api.set_joint_position(self.api.ankle_joint_handles[1], ankle_angle)
        self.api.set_joint_position(self.api.neck_joint_handle, neck_angle)
        self.api.set_joint_position(self.api.left_j2_handle, j2_angle)
        self.api.set_joint_position(self.api.right_j2_handle, j2_angle)
        self.api.set_joint_position(self.api.left_j3_handle, j3_angle)
        self.api.set_joint_position(self.api.right_j3_handle, j3_angle)
        self.api.set_joint_position(self.api.right_p4_handle, p4_position)
        self.api.set_joint_position(self.api.left_p4_handle, p4_position)

        # 推进仿真
        self.api.step()

    def move_prone_to_stand(self, dist):
        """实现从趴姿到站姿的过渡

        Args:
            dist: 归一化距离值 (0<=dist<=1)
        """
        t = dist % 1.0
        orientation = [0, 0, 0]

        # 使用插值函数计算方向和关节角度
        orientation[0] = self.interpolate_angle(PRONE_TO_STAND_WAYPOINTS[0], t)
        orientation[1] = self.interpolate_angle(PRONE_TO_STAND_WAYPOINTS[1], t)
        orientation[2] = self.interpolate_angle(PRONE_TO_STAND_WAYPOINTS[2], t)
        j2_angle = self.interpolate_angle(PRONE_TO_STAND_WAYPOINTS[3], t)
        j3_angle = self.interpolate_angle(PRONE_TO_STAND_WAYPOINTS[4], t)

        # 计算角度和位置
        j2_angle_alt = math.radians(360) - j2_angle - orientation[1]
        j3_angle_alt = j2_angle_alt - j3_angle
        p4_position = (
            (HEIGHT * math.cos(orientation[1]) - JOINT2_LENGTH * math.cos(j2_angle_alt))
            / math.cos(j3_angle_alt)
            - JOINT3_LENGTH
            - JOINT4_LENGTH
        )

        # 限制末端位置范围
        if p4_position > 0:
            p4_position = 0
        elif p4_position < -0.3:
            p4_position = -0.3

        # 防止绘图角度溢出
        if abs(j3_angle - self.api.get_joint_position(self.api.left_j3_handle)) > 2:
            p4_position = self.api.get_joint_position(self.api.right_p4_handle)
            j2_angle = 5.75592134
            j3_angle = self.api.get_joint_position(self.api.right_j3_handle)
            orientation[1] = 0

        # 记录运动数据
        test_data_point = np.array(
            [
                90 - math.degrees(orientation[1]),
                math.degrees(j2_angle),
                math.degrees(j3_angle),
                p4_position,
            ]
        ).reshape(4, 1)
        self.test_data = np.hstack((self.test_data, test_data_point))

        # 更新朝向和关节位置
        self.api.set_object_orientation(self.api.bill_handle, orientation)
        self.api.set_joint_position(self.api.left_j2_handle, j2_angle)
        self.api.set_joint_position(self.api.right_j2_handle, j2_angle)
        self.api.set_joint_position(self.api.left_j3_handle, j3_angle)
        self.api.set_joint_position(self.api.right_j3_handle, j3_angle)
        self.api.set_joint_position(self.api.right_p4_handle, p4_position)
        self.api.set_joint_position(self.api.left_p4_handle, p4_position)

        # 推进仿真
        self.api.step()

    def move_fall_to_stand(self, dist):
        """
        实现从摔倒到站姿的过渡
        """
        t = dist % 1.0
        orientation = [0, 0, 0]

        # 使用插值函数计算方向和关节角度
        orientation[0] = self.interpolate_angle(FALL_TO_STAND_WAYPOINTS[0], t)
        orientation[1] = self.interpolate_angle(FALL_TO_STAND_WAYPOINTS[1], t)
        orientation[2] = self.interpolate_angle(FALL_TO_STAND_WAYPOINTS[2], t)
        j2_angle = self.interpolate_angle(FALL_TO_STAND_WAYPOINTS[3], t)
        j3_angle = self.interpolate_angle(FALL_TO_STAND_WAYPOINTS[4], t)

        j2_angle_alt = j2_angle - abs(orientation[1])
        j3_angle_alt = j3_angle + j2_angle_alt

        p4_position = (
            (HEIGHT * math.cos(orientation[1]) - JOINT2_LENGTH * math.cos(j2_angle_alt))
            / math.cos(j3_angle_alt)
            - JOINT3_LENGTH
            - JOINT4_LENGTH
        )
        # 限制末端端位置范围
        if p4_position > 0:
            p4_position = 0
        elif p4_position < -0.3:
            p4_position = -0.3
         # 防止绘图角度溢出
        if abs(j2_angle - self.api.get_joint_position(self.api.left_j2_handle)) > 0.5:
            p4_position = -0.0966
            j2_angle = 0.5246
            j3_angle = 0
            orientation[1] = 0
        # 记录运动数据
        test_data_point = np.array(
            [
                math.degrees(orientation[1]),
                math.degrees(j2_angle),
                math.degrees(j3_angle),
                p4_position,
            ]
        ).reshape(4, 1)
        self.test_data = np.hstack((self.test_data, test_data_point))

        # 更新朝向和关节位置
        self.api.set_object_orientation(self.api.bill_handle, orientation)
        self.api.set_joint_position(self.api.left_j2_handle, j2_angle)
        self.api.set_joint_position(self.api.right_j2_handle, j2_angle)
        self.api.set_joint_position(self.api.left_j3_handle, j3_angle)
        self.api.set_joint_position(self.api.right_j3_handle, j3_angle)
        self.api.set_joint_position(self.api.right_p4_handle, p4_position)
        self.api.set_joint_position(self.api.left_p4_handle, p4_position)

        # 推进仿真
        self.api.step()
