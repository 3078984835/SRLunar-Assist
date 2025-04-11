import time
from Call_API import CallAPI
from Sport import Sport
from Plot import Plot

def main():
    # 实例化 API 客户端对象
    api = CallAPI()

    # 如果仿真没有停止，则先停止仿真
    if api.get_simulation_state() != api.sim.simulation_stopped:
        api.stop_simulation()
        while api.get_simulation_state() != api.sim.simulation_stopped:
            pass

    # 加载场景（请根据实际情况修改路径）
    scene_path = r"D:\STUDY\BS\CoopeliaSimApi\body-protect-machine\test006.ttt"
    api.load_scene(scene_path)

    # 获取所有需要的关节句柄
    api.get_handles()
    # 设置零重力环境
    api.sim.setArrayParam(api.sim.arrayparam_gravity, [0.0, 0.0, 0.0])
    # 归零关节位置
    api.zero_joints()

    # 开始仿真
    api.start_simulation()

    # 实例化运动类，传入 API 对象
    sport = Sport(api)

    # 模拟运动主循环：dist 从 0 增加到 60（视具体仿真需求调整）
    dist = 0.0
    step_size = 0.01
    while dist < 60 + 1e-9:
        # 记录当前仿真时间
        sport.current_time.append(api.get_simulation_time())
        # Bill 的运动函数
        sport.move_Bill(dist)
        # 机器人外肢体运动函数
        sport.move_robot(dist)
        dist += step_size

        # 如果仿真因其他原因停止，则退出循环
        if api.get_simulation_state() == api.sim.simulation_stopped:
            print("仿真已停止!")
            break

    # 停止仿真
    api.stop_simulation()

    # 绘制图形
    plotter = Plot()
    plotter.plot_robot(sport.current_time, sport.robot_massage)

if __name__ == "__main__":
    main()
