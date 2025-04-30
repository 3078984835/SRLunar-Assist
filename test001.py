import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import math
import zhplot

# 参数
a = 1 / 3 * 210
b = -210 * 1.62 * 0.9116
c = 2 * 0.9116 * 186.39
y0 = math.radians(60)  # 初始角度
print(y0)


# 定义微分方程组
def ode_system(t, y):
    dydx = y[1]
    dzdx = (c + b * np.sin(y0 + y[0])) / a
    return [dydx, dzdx]


# 初值条件
y_initial = [y0, -0.633]  # y(0), y'(0)

# 时间区间
t_span = (0, 2.5)
t_eval = np.linspace(t_span[0], t_span[1], 500)

# 求解
solution = solve_ivp(ode_system, t_span, y_initial, t_eval=t_eval, method="RK45")

# 绘制结果
plt.plot(solution.t, solution.y[0])
plt.plot(solution.t, solution.y[1])
plt.xlabel("t(s)")
plt.ylabel("θ(rad), θ'(rad/s)")
plt.title("四阶龙格库塔解法所求运动方程的解")
plt.grid()
plt.legend(["θ(t)", "θ'(t)"])
plt.show()
