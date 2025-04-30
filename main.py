"""
主程序入口 - 协调各模块完成仿真任务
"""

import time
from constants import MODE_WALK, MODE_SIT, MODE_PRONE, SCENE_PATHS, MODE_FALL
from simulation_api import SimulationAPI
from motion import Motion
from visualization import Visualization


def initialize_simulation(mode):
    """初始化仿真环境

    Args:
        mode: 仿真模式，可选值为 walk、sit、prone

    Returns:
        api: 初始化后的SimulationAPI实例
    """
    # 实例化API客户端
    api = SimulationAPI()

    # 如果仿真没有停止，则先停止仿真
    if api.get_simulation_state() != api.sim.simulation_stopped:
        api.stop_simulation()
        while api.get_simulation_state() != api.sim.simulation_stopped:
            pass

    # 加载对应模式的场景
    api.load_scene(SCENE_PATHS[mode])

    # 获取所有需要的关节句柄
    api.get_handles()

    # 设置零重力环境
    api.sim.setArrayParam(api.sim.arrayparam_gravity, [0.0, 0.0, 0.0])

    # 根据模式初始化关节位置
    if mode == MODE_WALK:
        api.zero_joints_walk()
    elif mode == MODE_SIT:
        api.zero_joints_sit()
    elif mode == MODE_PRONE:
        api.zero_joints_prone()
    elif mode == MODE_FALL:
        api.zero_joints_fall()
    else:
        raise ValueError("未定义模式： {}".format(mode))


    # 开始仿真
    api.start_simulation()
    return api


def run_walk_simulation(api):
    """运行步行仿真

    Args:
        api: SimulationAPI实例
    """
    # 实例化运动控制器
    motion = Motion(api)

    # 模拟运动主循环
    dist = 0.0
    step_size = 0.001
    while dist < 2 + 1e-9:
        # 记录当前仿真时间
        motion.current_time.append(api.get_simulation_time())

        # 控制Bill的运动
        motion.move_bill(dist)

        # 控制机器人外肢体运动
        motion.move_robot(dist)

        # 更新距离
        dist += step_size

        # 如果仿真因其他原因停止，则退出循环
        if api.get_simulation_state() == api.sim.simulation_stopped:
            print("仿真已停止!")
            break

    # 停止仿真
    api.stop_simulation()

    # 绘制图形
    viz = Visualization()
    viz.plot_robot_walk(motion.current_time, motion.robot_data)
    viz.plot_bill_position(motion.current_time, motion.bill_position_data)


def run_sit_to_stand_simulation(api):
    """运行从坐姿到站姿的仿真

    Args:
        api: SimulationAPI实例
    """
    # 实例化运动控制器
    motion = Motion(api)

    # 模拟从坐姿到站姿的过程
    dist = 0.0
    step_size = 0.001
    while dist < 1 + 1e-9:
        # 记录当前仿真时间
        motion.current_time.append(api.get_simulation_time())

        # 控制从坐姿到站姿的过渡
        motion.move_sit_to_stand(dist)

        # 更新距离
        dist += step_size

        # 如果仿真因其他原因停止，则退出循环
        if api.get_simulation_state() == api.sim.simulation_stopped:
            print("仿真已停止!")
            break

    # 停止仿真
    api.stop_simulation()


def run_prone_to_stand_simulation(api):
    """运行从趴姿到站姿的仿真

    Args:
        api: SimulationAPI实例
    """
    # 实例化运动控制器
    motion = Motion(api)

    # 模拟从趴姿到站姿的过程
    dist = 0.0
    step_size = 0.001
    while dist < 1 + 1e-9:
        # 记录当前仿真时间
        motion.current_time.append(api.get_simulation_time())

        # 控制从趴姿到站姿的过渡
        motion.move_prone_to_stand(dist)

        # 更新距离
        dist += step_size

        # 如果仿真因其他原因停止，则退出循环
        if api.get_simulation_state() == api.sim.simulation_stopped:
            print("仿真已中断!")
            break        
    # 停止仿真
    api.stop_simulation()
    print("仿真已停止!")

    # 绘制图形
    viz = Visualization()
    viz.plot_robot_prone(motion.current_time, motion.test_data)


def run_fall_to_stand_simulation(api):
    """运行从摔倒到站姿的仿真

    Args:
        api: SimulationAPI实例
    """
    # 实例化运动控制器
    motion = Motion(api)

    # 模拟从摔倒到站姿的过程
    dist = 0.0
    step_size = 0.001
    while dist < 1 + 1e-9:
        # 记录当前仿真时间
        motion.current_time.append(api.get_simulation_time())

        # 控制从摔倒到站姿的过渡
        motion.move_fall_to_stand(dist)

        # 更新距离
        dist += step_size

        # 如果仿真因其他原因停止，则退出循环
        if api.get_simulation_state() == api.sim.simulation_stopped:
            print("仿真已中断!")
            break

    # 停止仿真
    api.stop_simulation()
    print("仿真已停止!")

    # 绘制图形
    viz = Visualization()
    viz.plot_robot_prone(motion.current_time, motion.test_data)


def main():
    """主函数，程序入口"""
    # 选择模拟模式
    mode = MODE_FALL  # 可选值为 walk、sit 或 prone fall

    try:
        # 初始化仿真
        api = initialize_simulation(mode)

        # 根据模式运行不同的仿真
        if mode == MODE_WALK:
            print(mode)
            run_walk_simulation(api)            
        elif mode == MODE_SIT:
            print(mode)
            run_sit_to_stand_simulation(api)            
        elif mode == MODE_PRONE:
            print(mode)
            run_prone_to_stand_simulation(api)
        elif mode == MODE_FALL:
            print(mode)
            run_fall_to_stand_simulation(api)            
        else:
            print(f"无效的模式: {mode}，请选择 'walk'、'sit' 或 'prone'。")
    except Exception as e:
        print(f"仿真过程中出现错误: {e}")
    finally:
        # 确保仿真正常停止
        if (
            "api" in locals()
            and api.get_simulation_state() != api.sim.simulation_stopped
        ):
            api.stop_simulation()


if __name__ == "__main__":
    main()
