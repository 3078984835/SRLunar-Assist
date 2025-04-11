import math
import numpy as np
from config import LEG_WAYPOINTS, KNEE_WAYPOINTS, ANKLE_WAYPOINTS, SHOULDER_WAYPOINTS, ELBOW_WAYPOINTS, AMPLITUDE_COEFFICIENT, JOINT2_LENGTH, JOINT3_LENGTH, JOINT4_LENGTH, HEIGHT

class Sport:
    def __init__(self, api):
        # 接受 CallAPI 类的实例用于调用仿真接口
        self.api = api
        # 存储当前仿真时间，机器人数据和 Bill 数据
        self.current_time = []
        self.robot_massage = np.zeros((8, 0))  # 8行，动态添加列
        self.Bill_massage = np.zeros((13, 0))   # 13行

    def interpolate_angle(self, waypoints, t):
        """给定关键帧列表和归一化时间 t (0<=t<=1)，返回线性插值后的角度"""
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
        # 计算右腿和左腿的相位 t1 和 t2
        t1 = dist % 1.0
        t2 = (t1 + 0.5) % 1.0

        # 通过线性插值计算腿部和膝关节角度
        rightLegAngle = self.interpolate_angle(LEG_WAYPOINTS, t1)
        rightKneeAngle = self.interpolate_angle(KNEE_WAYPOINTS, t1)
        leftLegAngle = self.interpolate_angle(LEG_WAYPOINTS, t2)
        leftKneeAngle = self.interpolate_angle(KNEE_WAYPOINTS, t2)

        # 根据用户调节系数对角度进行增幅
        rightLegAngle_adj = rightLegAngle * (1 + AMPLITUDE_COEFFICIENT)
        rightKneeAngle_adj = rightKneeAngle * (1 + AMPLITUDE_COEFFICIENT)
        leftLegAngle_adj = leftLegAngle * (1 + AMPLITUDE_COEFFICIENT)
        leftKneeAngle_adj = leftKneeAngle * (1 + AMPLITUDE_COEFFICIENT)

        # 设置机器人外肢体关节角度
        self.api.set_joint_position(self.api.RJ2Handle, rightLegAngle_adj)
        self.api.set_joint_position(self.api.RJ3Handle, rightKneeAngle_adj)
        self.api.set_joint_position(self.api.LJ2Handle, leftLegAngle_adj)
        self.api.set_joint_position(self.api.LJ3Handle, leftKneeAngle_adj)

        # 计算末端节位置（采用不同角度范围计算）
        rightLength = ((HEIGHT - JOINT2_LENGTH * math.cos(abs(rightLegAngle_adj))) /
            (math.cos(abs(rightLegAngle_adj) - abs(rightKneeAngle_adj))) - JOINT3_LENGTH - JOINT4_LENGTH)
        leftLength = ((HEIGHT - JOINT2_LENGTH * math.cos(abs(leftLegAngle_adj))) /
            (math.cos(abs(leftLegAngle_adj) - abs(leftKneeAngle_adj))) - JOINT3_LENGTH - JOINT4_LENGTH)

        leftLength_ = ((HEIGHT - JOINT2_LENGTH * math.cos(abs(rightLegAngle_adj))) /
            (math.cos(abs(rightLegAngle_adj) + abs(rightKneeAngle_adj))) - JOINT3_LENGTH - JOINT4_LENGTH)
        rightLength_ = ((HEIGHT - JOINT2_LENGTH * math.cos(abs(leftLegAngle_adj))) /
            (math.cos(abs(leftLegAngle_adj) + abs(leftKneeAngle_adj))) - JOINT3_LENGTH - JOINT4_LENGTH)

        # 设置末端节位置
        if t1 < 0.5:
            # 左腿前进，右腿后退
            self.api.set_joint_position(self.api.RP4Handle, -0.3)
            self.api.set_joint_position(self.api.LP4Handle, leftLength)
            if leftLegAngle_adj > 0:
                self.api.set_joint_position(self.api.LP4Handle, rightLength_)
        else:
            # 左腿后退，右腿前进
            self.api.set_joint_position(self.api.RP4Handle, rightLength)
            self.api.set_joint_position(self.api.LP4Handle, -0.3)
            if rightLegAngle_adj > 0:
                self.api.set_joint_position(self.api.LP4Handle, leftLength_)

        # 为了便于调试，将角度转换为度数
        leftLegAngle_deg = math.degrees(self.api.get_joint_position(self.api.RJ2Handle))
        leftKneeAngle_deg = math.degrees(self.api.get_joint_position(self.api.RJ3Handle))
        rightLegAngle_deg = math.degrees(self.api.get_joint_position(self.api.LJ2Handle))
        rightKneeAngle_deg = math.degrees(self.api.get_joint_position(self.api.LJ3Handle))

        # 记录机器人关节数据供后续绘图使用
        robot_data = np.array([
            0, 0, leftLegAngle_deg, rightLegAngle_deg,
            leftKneeAngle_deg, rightKneeAngle_deg, leftLength, rightLength
        ]).reshape(8, 1)
        self.robot_massage = np.hstack((self.robot_massage, robot_data))

        # 推进仿真一步
        self.api.step()

    def move_Bill(self, dist):
        t1 = dist % 1.0
        t2 = (t1 + 0.5) % 1.0

        leftLegAngle = self.interpolate_angle(LEG_WAYPOINTS, t1)
        leftKneeAngle = self.interpolate_angle(KNEE_WAYPOINTS, t1)
        rightLegAngle = self.interpolate_angle(LEG_WAYPOINTS, t2)
        rightKneeAngle = self.interpolate_angle(KNEE_WAYPOINTS, t2)

        self.api.set_joint_position(self.api.legJointHandles[0], leftLegAngle)
        self.api.set_joint_position(self.api.kneeJointHandles[0], leftKneeAngle)
        self.api.set_joint_position(self.api.legJointHandles[1], rightLegAngle)
        self.api.set_joint_position(self.api.kneeJointHandles[1], rightKneeAngle)

        leftAnkleAngle = self.interpolate_angle(ANKLE_WAYPOINTS, t1)
        rightAnkleAngle = self.interpolate_angle(ANKLE_WAYPOINTS, t2)
        self.api.set_joint_position(self.api.ankleJointHandles[0], leftAnkleAngle)
        self.api.set_joint_position(self.api.ankleJointHandles[1], rightAnkleAngle)

        leftShoulderAngle = self.interpolate_angle(SHOULDER_WAYPOINTS, t1)
        rightShoulderAngle = self.interpolate_angle(SHOULDER_WAYPOINTS, t2)
        self.api.set_joint_position(self.api.shoulderJointHandles[0], leftShoulderAngle)
        self.api.set_joint_position(self.api.shoulderJointHandles[1], rightShoulderAngle)

        leftElbowAngle = self.interpolate_angle(ELBOW_WAYPOINTS, t1)
        rightElbowAngle = self.interpolate_angle(ELBOW_WAYPOINTS, t2)
        self.api.set_joint_position(self.api.elbowJointHandles[0], leftElbowAngle)
        self.api.set_joint_position(self.api.elbowJointHandles[1], rightElbowAngle)

        # 将角度转换为度
        leftLegAngle_deg = math.degrees(self.api.get_joint_position(self.api.legJointHandles[0]))
        rightLegAngle_deg = math.degrees(self.api.get_joint_position(self.api.legJointHandles[1]))
        leftKneeAngle_deg = math.degrees(self.api.get_joint_position(self.api.kneeJointHandles[0]))
        rightKneeAngle_deg = math.degrees(self.api.get_joint_position(self.api.kneeJointHandles[1]))
        leftAnkleAngle_deg = math.degrees(self.api.get_joint_position(self.api.ankleJointHandles[0]))
        rightAnkleAngle_deg = math.degrees(self.api.get_joint_position(self.api.ankleJointHandles[1]))
        leftShoulderAngle_deg = math.degrees(self.api.get_joint_position(self.api.shoulderJointHandles[0]))
        rightShoulderAngle_deg = math.degrees(self.api.get_joint_position(self.api.shoulderJointHandles[1]))
        leftElbowAngle_deg = math.degrees(self.api.get_joint_position(self.api.elbowJointHandles[0]))
        rightElbowAngle_deg = math.degrees(self.api.get_joint_position(self.api.elbowJointHandles[1]))
        neckAngle_deg = math.degrees(self.api.get_joint_position(self.api.neckJointHandle))

        bill_data = np.array([
            0, 0, leftLegAngle_deg, rightLegAngle_deg,
            leftKneeAngle_deg, rightKneeAngle_deg,
            leftAnkleAngle_deg, rightAnkleAngle_deg,
            leftShoulderAngle_deg, rightShoulderAngle_deg,
            leftElbowAngle_deg, rightElbowAngle_deg,
            neckAngle_deg,
        ]).reshape(13, 1)
        self.Bill_massage = np.hstack((self.Bill_massage, bill_data))
