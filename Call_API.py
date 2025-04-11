from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class CallAPI:
    def __init__(self):
        # 初始化 ZeroMQ 远程API 客户端并获取 sim 对象
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.sim.setStepping(True)

        # 定义各个关节的句柄（后续通过 get_handles 获取）
        self.RJ2Handle = None
        self.LJ2Handle = None
        self.RJ3Handle = None
        self.LJ3Handle = None
        self.RP4Handle = None
        self.LP4Handle = None
        self.BillHandle = None
        self.legJointHandles = []
        self.kneeJointHandles = []
        self.ankleJointHandles = []
        self.shoulderJointHandles = []
        self.elbowJointHandles = []
        self.neckJointHandle = None
        self.pathHandle = None

    def load_scene(self, scene_path):
        self.sim.loadScene(scene_path)

    def start_simulation(self):
        self.sim.startSimulation()

    def stop_simulation(self):
        self.sim.stopSimulation()

    def get_simulation_state(self):
        return self.sim.getSimulationState()

    def step(self):
        self.sim.step()

    def get_simulation_time(self):
        return self.sim.getSimulationTime()

    def set_joint_position(self, handle, position):
        self.sim.setJointPosition(handle, position)

    def get_joint_position(self, handle):
        return self.sim.getJointPosition(handle)

    def get_handles(self):
        # 获取机器人外肢体关节句柄
        self.RJ2Handle = self.sim.getObject("/RJ2")
        self.LJ2Handle = self.sim.getObject("/LJ2")
        self.RJ3Handle = self.sim.getObject("/RJ3")
        self.LJ3Handle = self.sim.getObject("/LJ3")
        self.RP4Handle = self.sim.getObject("/RP4")
        self.LP4Handle = self.sim.getObject("/LP4")
        # 获取 Bill 关节句柄
        self.BillHandle = self.sim.getObject("/Bill")
        self.legJointHandles = [
            self.sim.getObject("./leftLegJoint"),
            self.sim.getObject("./rightLegJoint"),
        ]
        self.kneeJointHandles = [
            self.sim.getObject("./leftKneeJoint"),
            self.sim.getObject("./rightKneeJoint"),
        ]
        self.ankleJointHandles = [
            self.sim.getObject("./leftAnkleJoint"),
            self.sim.getObject("./rightAnkleJoint"),
        ]
        self.shoulderJointHandles = [
            self.sim.getObject("./leftShoulderJoint"),
            self.sim.getObject("./rightShoulderJoint"),
        ]
        self.elbowJointHandles = [
            self.sim.getObject("./leftElbowJoint"),
            self.sim.getObject("./rightElbowJoint"),
        ]
        self.neckJointHandle = self.sim.getObject("./neck")
        self.pathHandle = self.sim.getObject("./path")

    def zero_joints(self):
        # 初始化机械臂关节位置
        self.set_joint_position(self.RJ2Handle, 0)
        self.set_joint_position(self.LJ2Handle, 0)
        self.set_joint_position(self.RJ3Handle, 0)
        self.set_joint_position(self.LJ3Handle, 0)
        self.set_joint_position(self.RP4Handle, -0.3)
        self.set_joint_position(self.LP4Handle, -0.3)
        # 初始化 Bill 各关节位置
        self.set_joint_position(self.legJointHandles[0], 0)
        self.set_joint_position(self.legJointHandles[1], 0)
        self.set_joint_position(self.kneeJointHandles[0], 0)
        self.set_joint_position(self.kneeJointHandles[1], 0)
        self.set_joint_position(self.ankleJointHandles[0], 0)
        self.set_joint_position(self.ankleJointHandles[1], 0)
        self.set_joint_position(self.shoulderJointHandles[0], 0)
        self.set_joint_position(self.shoulderJointHandles[1], 0)
        self.set_joint_position(self.elbowJointHandles[0], 0)
        self.set_joint_position(self.elbowJointHandles[1], 0)
        self.set_joint_position(self.neckJointHandle, 0)
