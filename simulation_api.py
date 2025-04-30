"""
仿真API模块 - 封装CoppeliaSim仿真环境的API调用
"""

import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class SimulationAPI:
    """CoppeliaSim仿真环境API封装类"""

    def __init__(self):
        """初始化仿真环境连接"""
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.sim.setStepping(True)

        # 句柄初始化
        self.init_handles()

    def init_handles(self):
        """初始化所有句柄为None"""
        # 机械臂关节句柄
        self.right_j2_handle = None
        self.left_j2_handle = None
        self.right_j3_handle = None
        self.left_j3_handle = None
        self.right_p4_handle = None
        self.left_p4_handle = None

        # Bill关节句柄
        self.bill_handle = None
        self.leg_joint_handles = []
        self.knee_joint_handles = []
        self.ankle_joint_handles = []
        self.shoulder_joint_handles = []
        self.elbow_joint_handles = []
        self.neck_joint_handle = None
        self.path_handle = None

    def load_scene(self, scene_path):
        """加载场景"""
        self.sim.loadScene(scene_path)

    def start_simulation(self):
        """开始仿真"""
        self.sim.startSimulation()

    def stop_simulation(self):
        """停止仿真"""
        self.sim.stopSimulation()

    def get_simulation_state(self):
        """获取仿真状态"""
        return self.sim.getSimulationState()

    def step(self):
        """推进仿真一步"""
        self.sim.step()

    def get_simulation_time(self):
        """获取当前仿真时间"""
        return self.sim.getSimulationTime()

    def set_joint_position(self, handle, position):
        """设置关节位置"""
        self.sim.setJointPosition(handle, position)

    def get_joint_position(self, handle):
        """获取关节位置"""
        return self.sim.getJointPosition(handle)

    def get_object_orientation(self, handle):
        """获取对象方向"""
        return self.sim.getObjectOrientation(handle)

    def set_object_orientation(self, handle, orientation):
        """设置对象方向"""
        self.sim.setObjectOrientation(handle, orientation)

    def get_object_position(self, handle):
        """获取对象位置"""
        return self.sim.getObjectPosition(handle)

    def set_object_position(self, handle, position):
        """设置对象位置"""
        self.sim.setObjectPosition(handle, position)

    def get_object(self, object_name):
        """通过名称获取对象句柄"""
        return self.sim.getObject(object_name)

    def get_object_pose(self, handle):
        """获取对象姿态"""
        return self.sim.getObjectPose(handle)

    def multiply_vector(self, pose, vector):
        """通过姿态变换向量"""
        return self.sim.multiplyVector(pose, vector)

    def get_path_lengths(self, path_positions, dimension):
        """获取路径长度"""
        return self.sim.getPathLengths(path_positions, dimension)

    def get_path_interpolated_config(self, path_positions, path_lengths, distance):
        """获取路径插值配置"""
        return self.sim.getPathInterpolatedConfig(
            path_positions, path_lengths, distance
        )

    def unpack_double_table(self, data):
        """解包双精度数据表"""
        return self.sim.unpackDoubleTable(data)

    def read_custom_buffer_data(self, handle, tag):
        """读取自定义缓冲区数据"""
        return self.sim.readCustomBufferData(handle, tag)

    def get_handles(self):
        """获取所有需要的句柄"""
        # 获取机器人外肢体关节句柄
        self.right_j2_handle = self.get_object("/RJ2")
        self.left_j2_handle = self.get_object("/LJ2")
        self.right_j3_handle = self.get_object("/RJ3")
        self.left_j3_handle = self.get_object("/LJ3")
        self.right_p4_handle = self.get_object("/RP4")
        self.left_p4_handle = self.get_object("/LP4")

        # 获取Bill关节句柄
        self.bill_handle = self.get_object("/Bill")
        self.leg_joint_handles = [
            self.get_object("./leftLegJoint"),
            self.get_object("./rightLegJoint"),
        ]
        self.knee_joint_handles = [
            self.get_object("./leftKneeJoint"),
            self.get_object("./rightKneeJoint"),
        ]
        self.ankle_joint_handles = [
            self.get_object("./leftAnkleJoint"),
            self.get_object("./rightAnkleJoint"),
        ]
        self.shoulder_joint_handles = [
            self.get_object("./leftShoulderJoint"),
            self.get_object("./rightShoulderJoint"),
        ]
        self.elbow_joint_handles = [
            self.get_object("./leftElbowJoint"),
            self.get_object("./rightElbowJoint"),
        ]
        self.neck_joint_handle = self.get_object("./neck")
        self.path_handle = self.get_object("./path")

    def zero_joints_walk(self):
        """初始化所有关节到行走姿势的零位"""
        # 初始化机械臂关节位置
        self.set_joint_position(self.right_j2_handle, 0)
        self.set_joint_position(self.left_j2_handle, 0)
        self.set_joint_position(self.right_j3_handle, 0)
        self.set_joint_position(self.left_j3_handle, 0)
        self.set_joint_position(self.right_p4_handle, -0.3)
        self.set_joint_position(self.left_p4_handle, -0.3)

        # 初始化Bill各关节位置
        self.set_joint_position(self.leg_joint_handles[0], 0)
        self.set_joint_position(self.leg_joint_handles[1], 0)
        self.set_joint_position(self.knee_joint_handles[0], 0)
        self.set_joint_position(self.knee_joint_handles[1], 0)
        self.set_joint_position(self.ankle_joint_handles[0], 0)
        self.set_joint_position(self.ankle_joint_handles[1], 0)
        self.set_joint_position(self.shoulder_joint_handles[0], 0)
        self.set_joint_position(self.shoulder_joint_handles[1], 0)
        self.set_joint_position(self.elbow_joint_handles[0], 0)
        self.set_joint_position(self.elbow_joint_handles[1], 0)
        self.set_joint_position(self.neck_joint_handle, 0)

    def zero_joints_sit(self):
        """初始化所有关节到坐姿的零位"""
        # 初始化机械臂关节位置
        self.set_joint_position(self.right_j2_handle, math.radians(60))
        self.set_joint_position(self.left_j2_handle, math.radians(60))
        self.set_joint_position(self.right_j3_handle, math.radians(-90))
        self.set_joint_position(self.left_j3_handle, math.radians(-90))
        self.set_joint_position(self.right_p4_handle, -0.3)
        self.set_joint_position(self.left_p4_handle, -0.3)

        # 初始化Bill各关节位置
        self.set_joint_position(self.leg_joint_handles[0], -1.57)
        self.set_joint_position(self.leg_joint_handles[1], -1.57)
        self.set_joint_position(self.knee_joint_handles[0], 1.57)
        self.set_joint_position(self.knee_joint_handles[1], 1.57)
        self.set_joint_position(self.ankle_joint_handles[0], 0)
        self.set_joint_position(self.ankle_joint_handles[1], 0)
        self.set_joint_position(self.shoulder_joint_handles[0], 0)
        self.set_joint_position(self.shoulder_joint_handles[1], 0)
        self.set_joint_position(self.elbow_joint_handles[0], 0)
        self.set_joint_position(self.elbow_joint_handles[1], 0)
        self.set_joint_position(self.neck_joint_handle, 0)

    def zero_joints_prone(self):
        """初始化所有关节到趴姿的零位"""
        # 初始化机械臂关节位置
        self.set_joint_position(self.right_j2_handle, math.radians(120))
        self.set_joint_position(self.left_j2_handle, math.radians(120))
        self.set_joint_position(self.right_j3_handle, math.radians(150))
        self.set_joint_position(self.left_j3_handle, math.radians(150))
        self.set_joint_position(self.right_p4_handle, -0.3)
        self.set_joint_position(self.left_p4_handle, -0.3)

        # 初始化Bill各关节位置
        self.set_joint_position(self.leg_joint_handles[0], 0)
        self.set_joint_position(self.leg_joint_handles[1], 0)
        self.set_joint_position(self.knee_joint_handles[0], 0)
        self.set_joint_position(self.knee_joint_handles[1], 0)
        self.set_joint_position(self.ankle_joint_handles[0], 0)
        self.set_joint_position(self.ankle_joint_handles[1], 0)
        self.set_joint_position(self.shoulder_joint_handles[0], 0)
        self.set_joint_position(self.shoulder_joint_handles[1], 0)
        self.set_joint_position(self.elbow_joint_handles[0], 0)
        self.set_joint_position(self.elbow_joint_handles[1], 0)
        self.set_joint_position(self.neck_joint_handle, 0)

        # 设置Bill的初始朝向
        self.set_object_orientation(self.bill_handle, [0, math.radians(90), 0])

    def zero_joints_fall(self):
        # 设置Bill的初始朝向
        self.set_object_orientation(self.bill_handle, [0, 0, 0])

        # 初始化机械臂关节位置
        self.set_joint_position(self.right_j2_handle, 0)
        self.set_joint_position(self.left_j2_handle, 0)
        self.set_joint_position(self.right_j3_handle, 0)
        self.set_joint_position(self.left_j3_handle, 0)
        self.set_joint_position(self.right_p4_handle, -0.3)
        self.set_joint_position(self.left_p4_handle, -0.3)

        # 初始化Bill各关节位置
        self.set_joint_position(self.leg_joint_handles[0], 0)
        self.set_joint_position(self.leg_joint_handles[1], 0)
        self.set_joint_position(self.knee_joint_handles[0], 0)
        self.set_joint_position(self.knee_joint_handles[1], 0)
        self.set_joint_position(self.ankle_joint_handles[0], 0)
        self.set_joint_position(self.ankle_joint_handles[1], 0)
        self.set_joint_position(self.shoulder_joint_handles[0], 0)
        self.set_joint_position(self.shoulder_joint_handles[1], 0)
        self.set_joint_position(self.elbow_joint_handles[0], 0)
        self.set_joint_position(self.elbow_joint_handles[1], 0)
        self.set_joint_position(self.neck_joint_handle, 0)
