import numpy as np

# 全局常量定义

# 22个关键帧的时间比例（0到1均匀分布）
TIMINGS = [i / 21 for i in range(22)]

# 左/右髋关节（大腿根部关节）在步行周期内的22个关键帧角度值（单位：弧度）
LEG_WAYPOINTS = [
    0.237, 0.228, 0.175, -0.014, -0.133, -0.248, -0.323, -0.450,
    -0.450, -0.442, -0.407, -0.410, -0.377, -0.303, -0.178, -0.111,
    -0.010, 0.046, 0.104, 0.145, 0.188, 0.237,
]

# 左/右膝关节在步行周期内的22个关键帧角度值（单位：弧度）
KNEE_WAYPOINTS = [
    0.282, 0.403, 0.577, 0.929, 1.026, 1.047, 0.939, 0.664,
    0.440, 0.243, 0.230, 0.320, 0.366, 0.332, 0.269, 0.222,
    0.133, 0.089, 0.065, 0.073, 0.092, 0.282,
]

# 左/右踝关节在步行周期内的22个关键帧角度值（单位：弧度）
ANKLE_WAYPOINTS = [
    -0.133, 0.041, 0.244, 0.382, 0.304, 0.232, 0.266, 0.061,
    -0.090, -0.145, -0.043, 0.041, 0.001, 0.011, -0.099, -0.127,
    -0.121, -0.120, -0.107, -0.100, -0.090, -0.133,
]

# 左/右肩关节在步行周期内的22个关键帧角度值（单位：弧度）
SHOULDER_WAYPOINTS = [
    0.028, 0.043, 0.064, 0.078, 0.091, 0.102, 0.170, 0.245,
    0.317, 0.337, 0.402, 0.375, 0.331, 0.262, 0.188, 0.102,
    0.094, 0.086, 0.080, 0.051, 0.058, 0.028,
]

# 左/右肘关节在步行周期内的22个关键帧角度值（单位：弧度）
ELBOW_WAYPOINTS = [
    -1.148, -1.080, -1.047, -0.654, -0.517, -0.366, -0.242,
    -0.117, -0.078, -0.058, -0.031, -0.001, -0.009, 0.008, -0.108,
    -0.131, -0.256, -0.547, -0.709, -0.813, -1.014, -1.148,
]

# 用户可调节的幅度增大系数（例如 0.2 表示20%的增幅）
AMPLITUDE_COEFFICIENT = 0.2

# 外肢体关节长度及关节高度（单位：m）
JOINT2_LENGTH = 0.45
JOINT3_LENGTH = 0.3934
JOINT4_LENGTH = 0.44167
HEIGHT = 1.03
LONGST = 1.57
