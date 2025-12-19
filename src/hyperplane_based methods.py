import casadi as ca      # 引入CasADi库：这是一个强大的工具，用于自动微分和数值优化，类似于“让计算机帮你推导公式并求解”
import numpy as np       # 引入NumPy库：Python得力助手，用于处理矩阵、数组和数学运算
import matplotlib.pyplot as plt  # 引入Matplotlib：用于画图，把结果可视化
import time              # 引入time库：用于记录程序运行了多久

def vertices(A, b):
    """
    辅助函数：计算多面体的顶点。
    
    数学原理：
    一个凸多面体（比如矩形、多边形）可以用一组线性不等式 A * p <= b 表示。
    每一行 A[i]*p <= b[i] 实际上定义了一条直线的“半平面”。
    这个函数的作用就是找出这些直线围成的形状的“角点”（顶点），方便后续画图。
    
    参数:
        A: 约束矩阵，每一行代表一条边的法向量（垂直于边的方向）。
        b: 约束向量，代表边到原点的距离偏移。
    返回:
        V: 包含所有顶点坐标的矩阵 (2 x N)。
    """
    N = b.size
    V = np.full((2, N), np.nan)
    
    # 遍历每一对相邻的边，计算它们的交点
    for j in range(N - 1):
        if A[j, 1] == 0:
            # 如果第 j 条边是垂直线
            x = b[j] / A[j, 0]
            y = -(-b[j + 1] + A[j + 1, 0] * x) / A[j + 1, 1]
        elif A[j + 1, 1] == 0:
            # 如果第 j+1 条边是垂直线
            x = b[j + 1] / A[j + 1, 0]
            y = -(-b[j] + A[j, 0] * x) / A[j, 1]
        else:
            # 一般情况：解两个线性方程组求交点
            x = -(-b[j] / A[j, 1] + b[j + 1] / A[j + 1, 1]) / (A[j, 0] / A[j, 1] - A[j + 1, 0] / A[j + 1, 1])
            y = -(-b[j] + A[j, 0] * x) / A[j, 1]
        V[:, j] = [x, y]
    
    # 处理最后一个顶点（连接最后一条边和第一条边，闭合多边形）
    if A[N - 1, 1] == 0:
        x = b[N - 1] / A[N - 1, 0]
        y = -(-b[0] + A[0, 0] * x) / A[0, 1]
    elif A[0, 1] == 0:
        x = b[0] / A[0, 0]
        y = -(-b[N - 1] + A[N - 1, 0] * x) / A[N - 1, 1]
    else:
        x = -(-b[N - 1] / A[N - 1, 1] + b[0] / A[0, 1]) / (A[N - 1, 0] / A[N - 1, 1] - A[0, 0] / A[0, 1])
        y = -(-b[N - 1] + A[N - 1, 0] * x) / A[N - 1, 1]
    V[:, N - 1] = [x, y]

    return V


# ==========================================
# 1. 参数设置与环境定义
# ==========================================

# 任务配置
task = {'method': 1, 'TypeParking': 1, 'SolverType': 1, 'init_with_saved_guess': True}
print_solver_progress = True

# 优化算法参数
lamda_max = 1e5    # 分离超平面参数的上限（防止数值爆炸）
mu_max = 1e5
lambda_init = 1
mu_init = 1
eps = 1e-4         # 一个很小的数，用于防止除以0或保证严格大于0
N = 30             # 预测时域长度：我们将未来的轨迹切分成30个点
M = 5              # 积分步数：在每两个点之间，再细分5次计算物理运动，提高精度

# 车辆物理参数（单位：米）
tractor_len_front = 3.6  # 车头到前轴的距离
veh_len_rear = 1         # 后轴到车尾的距离
veh_width = 2            # 车宽
veh_wheelbase_1 = tractor_len_front - veh_len_rear # 轴距（前轮和后轮的距离）

# --- 车辆的几何形状定义 ---
# 我们用 "A*p <= b" 的形式来定义车辆为一个矩形盒子。
# Av0 是法向量，bv0 是距离限制。
Av0 = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])
bv0 = np.array([veh_width/2, tractor_len_front, veh_width/2, veh_len_rear])

# 计算车辆初始位置的顶点（用于画图）
# 局部坐标系下的4个角点
Vv0 = np.array([[tractor_len_front, tractor_len_front, -veh_len_rear, -veh_len_rear],
                [veh_width/2, -veh_width/2, -veh_width/2, veh_width/2]])

# --- 障碍物定义 ---
# 同样用 "A*p <= b" 定义障碍物。这里定义了4个障碍物。
"only one obstacle"
# Ao = [np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])]
# bo = [np.array([-3, 7, 10, 6])]
"two obstacles"
# Ao = [np.array([[0, 1], [1, 0], [0, -1], [-1, 0]]),
#       np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])]
# bo = [np.array([-3, 7, 10, 6]),
#       np.array([-2, 5, 8, 0])]
"four obstacles"
Ao = [np.array([[0, 1], [1, 0], [0, -1], [-1, 0]]),
      np.array([[0, 1], [1, 0], [0, -1], [-1, 0]]),
      np.array([[0, 1], [1, 0], [0, -1], [-1, 0]]),
      np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])]
bo = [np.array([-3, 7, 10, 6]),
      np.array([-2, 5, 8, 0]),
      np.array([10, 10, -7.5, -5]),
      np.array([10, 5, -3, 0])]

# --- 画布/边界定义 ---
# 车辆必须在这个范围内活动
Ac = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])
bc = np.array([10, 10, 10, 6])

# 画图用的坐标轴范围
xcmin, xcmax = -6, 10
ycmin, ycmax = -10, 10

# --- 约束条件 ---
# 状态约束：[x坐标, y坐标, 航向角theta, 速度v, 前轮转角delta]
# 例如：速度不能超过 5/3.6 m/s，转角不能超过 40度
xmin = np.array([xcmin, ycmin, -np.pi, -5/3.6, -40*np.pi/180])
xmax = np.array([xcmax, ycmax, np.pi, 5/3.6, 40*np.pi/180])

# 控制约束：[加速度a, 转向角速度omega]
umin = np.array([-1, -5*np.pi/180])
umax = np.array([1, 5*np.pi/180])

# 时间上限（最长允许跑多少秒）
tfmax = 300

# --- 起点和终点 ---
# 格式：[x, y, theta, v, delta]
x0 = np.array([0, 0, eps, 0, 0])           # 初始状态
xf = np.array([8.5, -7, np.pi/2 + eps, 0, 0])  # 目标状态

# ==========================================
# 2. 动力学模型构建 (CasADi符号编程)
# ==========================================

# 我们可以把这里想象成我们在告诉计算机“物理公式是什么”。
# 我们用符号（Symbolic）变量来代替具体的数字，这样CasADi后面才能求导。

# 定义时间轴
tau = np.linspace(0,1,N).T   
h = np.diff(tau[:2])[0]         

# 定义符号变量（Symbolic variables）
# 状态量
s = ca.SX.sym('s')      # x 坐标
d = ca.SX.sym('d')      # y 坐标
theta = ca.SX.sym('theta') # 航向角
v = ca.SX.sym('v')      # 速度
delta = ca.SX.sym('delta') # 前轮转角
x = ca.vertcat(s, d, theta, v, delta) # 把它们拼成一个向量

# 控制量
a = ca.SX.sym('a')      # 加速度
omega = ca.SX.sym('omega') # 转向角速度
tf = ca.SX.sym('tf')    # 总耗时（这也是一个我们要优化的变量！）
u = ca.vertcat(a, omega)

# --- 运动学方程 (Bicycle Model / 单车模型) ---
# 这是一个经典的车辆运动学模型：
# dx/dt = v * cos(theta)
# dy/dt = v * sin(theta)
# dtheta/dt = v * tan(delta) / L
# dv/dt = a
# ddelta/dt = omega
# 注意：这里乘以 tf 是因为我们把时间归一化到了 [0, 1] 区间，这是一种常用的技巧。
f = tf * ca.vertcat(v*ca.cos(theta), v*ca.sin(theta), v*ca.tan(delta)/veh_wheelbase_1, a, omega)

# --- 代价函数 (Cost Function) ---
# 我们希望什么？
# 1. 动作要小：加速度和打方向盘不要太猛 (100*a^2 + 100*omega^2) -> 使得驾驶平滑
# 2. 时间要短：尽快到达终点 (1 * tf)
Js = tf * (100*a**2 + 100*omega**2)  # 过程代价（平滑性）
Jf = 1 * tf                          # 最终代价（时间）

# 把上面的数学表达式打包成 CasADi 的函数对象，方便后面调用
fJs = ca.Function('f_Js', [x, ca.vertcat(u, tf)], [f, Js])
JF = ca.Function('Jf', [ca.vertcat(x, tf)], [Jf])

# ==========================================
# 3. 数值积分器 (Runge-Kutta 4)
# ==========================================
# 计算机不能处理连续时间，必须一步一步算。
# RK4 是一种非常精准的“步进”方法，用来预测下一时刻的状态。

DT = h / M  # 每个子步的时间长度
Xk = ca.SX.sym('Xk', x.size(1))
Uk = ca.SX.sym('Uk', u.size(1) + 1)
Xk_plus, Qk_plus = Xk, 0

# 执行 M 次小步积分，从当前状态推演到下一个离散时刻的状态
for j in range(M):
    k1, k1_q = fJs(Xk_plus, Uk)
    k2, k2_q = fJs(Xk_plus + DT/2 * k1, Uk)
    k3, k3_q = fJs(Xk_plus + DT/2 * k2, Uk)
    k4, k4_q = fJs(Xk_plus + DT * k3, Uk)
    Xk_plus = Xk_plus + DT/6 * (k1 + 2*k2 + 2*k3 + k4)
    Qk_plus = Qk_plus + DT/6 * (k1_q + 2*k2_q + 2*k3_q + k4_q)

# 积分器函数：输入当前状态和控制 -> 输出下一步状态和这一步的累积代价
F = ca.Function('F', [Xk, Uk], [Xk_plus, Qk_plus])

# ==========================================
# 4. 优化问题建模 (Optimization Setup)
# ==========================================

opti = ca.Opti()  # 创建一个优化问题实例

nx, nu = x0.size, u.size(1)

# --- 决策变量 ---
# 我们要求解的是：
# 1. 全部的状态轨迹 X (N+1 个点)
# 2. 每一步的控制输入 U (N 个点)
# 3. 总时间 Tf
X = opti.variable(nx, N+1)
U = opti.variable(nu, N)
Tf = opti.variable(1, 1)

# --- 初始猜测 (Initial Guess) ---
# 告诉求解器“大概”的解在哪里，能加速求解。
# 这里我们简单地猜测车辆是做匀速直线运动直接从起点跑到终点。
Xinit = np.zeros((nx, N+1))
Xinit[0, :] = np.linspace(x0[0], xf[0], N+1) # x 均匀插值
Xinit[1, :] = np.linspace(x0[1], xf[1], N+1) # y 均匀插值
Xinit[2, :] = np.linspace(x0[2], xf[2], N+1) # theta 均匀插值
Xinit[3, :] = np.linspace(x0[3], xf[3], N+1) # v 均匀插值
Xinit[4, :] = xmax[4]/5
Xinit[:, 0] = x0
Xinit[:, -1] = xf
Uinit = np.zeros((nu, N))
Uinit[0, :] = umax[0]/2

# 将猜测值传给求解器
opti.set_initial(X, Xinit)
opti.set_initial(U, Uinit)
opti.set_initial(Tf, 50) # 猜大概跑50秒

Aa_init = np.array([1,1])
bb_init = 1

# --- 目标函数与约束添加 ---
J = 0
opti.subject_to(X[:, 0] == x0)   # 约束：起点必须是 x0
opti.subject_to(X[:, -1] == xf)  # 约束：终点必须是 xf
opti.subject_to(0 <= Tf )        # 约束：时间必须是正的
opti.subject_to(Tf <= tfmax)     # 约束：时间不能超过上限
LA, Lb = [], []

if task['method'] == 1:  # 分离超平面方法
    # 遍历每一个时间步
    for k in range(N):
        # 1. 动力学约束：下一步的状态必须等于当前状态经过物理运动后的结果
        Xk_plus, Jk = F(X[:, k], ca.vertcat(U[:, k], Tf))
        J += Jk
        opti.subject_to(Xk_plus == X[:, k + 1])

        # 2. 物理极限约束：控制量和状态量不能超标
        opti.subject_to(umin <= U[:, k])
        opti.subject_to(U[:, k] <= umax)
        if k > 0:
            opti.subject_to(xmin <= X[:, k])
            opti.subject_to(X[:, k] <= xmax)

        # 3. 计算车身在世界坐标系的顶点位置
        # 旋转矩阵 R：将车身从局部坐标系旋转到世界坐标系
        R = ca.vertcat(ca.horzcat(ca.cos(X[2, k]), -ca.sin(X[2, k])),
                       ca.horzcat(ca.sin(X[2, k]), ca.cos(X[2, k])))
        T = X[0:2, k] # 平移向量：车的位置
        Vv = R @ Vv0 + T # 变换后的车身顶点

        # 4. 边界约束：车的所有顶点必须在画布范围内
        for v_idx in range(Vv.shape[1]):
            opti.subject_to(Ac @ Vv[:, v_idx] < bc)
            
        # 5. 避开障碍物：超平面分离定理 (Separating Hyperplane Theorem)
        # 核心思想：如果两个凸形（车和障碍物）不相撞，那么一定能找到一条直线，
        # 让车在直线一边，障碍物在另一边。
        for j in range(len(Ao)):
            Vo = vertices(Ao[j], bo[j])  # 获取当前障碍物的顶点
            
            # 定义这条神奇的直线（分离超平面）：Aa * p = bb
            # Aa 是法向量，bb 是偏移量。我们要把 Aa 和 bb 也作为优化变量，让求解器去找这条线。
            Aa = opti.variable(2, 1)
            bb = opti.variable(1, 1)
            bb_ones = ca.horzcat(*[bb for _ in range(Vv.shape[1])])
            
            opti.set_initial(Aa,Aa_init);
            opti.set_initial(bb,bb_init);
            
            # 限制 Aa 和 bb 的范围，防止数值过大
            opti.subject_to(-lamda_max <= Aa)
            opti.subject_to(Aa <= lamda_max)
            opti.subject_to(-lamda_max <= bb)
            opti.subject_to(bb <= lamda_max)
            
            # 约束：法向量 Aa 不能是零向量 (通过模长平方 >= eps 来实现)
            opti.subject_to(Aa.T @ Aa >= eps)
            
            # 约束：车的所有顶点都在直线的一侧 (Aa*p > bb)
            opti.subject_to(Aa.T @ Vv > bb_ones) 
            
            # 约束：障碍物的所有顶点都在直线的另一侧 (Aa*p < bb)
            opti.subject_to(Aa.T @ Vo < bb_ones)
            
            # 将变量保存起来，虽然这里没用，但在调试时可能有用
            LA.append(Aa)
            Lb.append(bb)
            
# opti.set_initial(LA,1);
# opti.set_initial(Lb,1);

# 添加终端代价
Jfinal = JF(ca.vertcat(X[:, -1], Tf))
J += Jfinal
opti.minimize(J) # 告诉求解器：我们要最小化 J

# ==========================================
# 5. 求解 (Solve)
# ==========================================

# 设置求解器选项
opts = {'expand': True} # expand=True 可以加速计算
if not print_solver_progress:
    opts['print_time'] = False
    opts['ipopt.print_level'] = 0
opti.solver('ipopt', opts) # 使用 IPOPT 求解器（它是一个非常著名的非线性优化求解器）

print("开始求解...")
start_time = time.time()
try:
    sol = opti.solve() # --- 这里开始真正的计算，寻找最优解 ---
except:
    # 如果求解失败（比如找不到解），也会返回当前的某种结果以便调试
    print("求解器未能找到最优解，显示调试结果。")
    sol = opti.debug 

ctime = time.time() - start_time

# 提取结果（从符号变量变成数字）
Xopt = sol.value(X)   # 最优路径
Uopt = sol.value(U)   # 最优控制量
tfopt = sol.value(Tf) # 最优时间
Ef = np.sum(Uopt[0, :]**2 + Uopt[1, :]**2) # 计算一种“能量消耗”指标

# 打印结果
# sol.stats()['return_status'] 告诉你是否成功（例如 "Solve Succeeded"）
print(f"状态: {sol.stats()['return_status']}: 代价J={sol.value(J):.3f}, 用时tf={tfopt:.2f}s, 能量Ef={Ef:.3f}, 计算耗时={ctime:.2f}s")


# ==========================================
# 6. 可视化 (Plotting)
# ==========================================
# 把算出来的结果画出来，让我们能看到车是怎么跑的

plt.figure()
plt.grid(True)

# 1. 画画布边界（绿色框）
Vc = np.array([[xcmin, xcmax, xcmax, xcmin], [ycmin, ycmin, ycmax, ycmax]])
plt.plot(np.append(Vc[0, :len(bc)], Vc[0, 0]), np.append(Vc[1, :len(bc)], Vc[1, 0]), 'g', linewidth=3)

# 2. 画算出车辆中心轨迹（黑色线）
hh, = plt.plot(Xopt[0, :], Xopt[1, :], 'k', marker='o', label='Optimal path', linewidth=1.5)

# 3. 画障碍物（红色多边形）
Vo = np.empty((2, 0))
for i in range(len(bo)):
    v = vertices(Ao[i], bo[i]) # 算出顶点
    Vo = np.hstack((Vo, v))
    # 画线
    plt.plot(np.append(v[0, :len(bo[i])], v[0, 0]), np.append(v[1, :len(bo[i])], v[1, 0]), 'r', linewidth=2)

# 标记障碍物顶点（红星）
for j in range(Vo.shape[1]):
    plt.plot(Vo[0, j], Vo[1, j], 'r*', linewidth=2)

# 4. 画车辆在每个时刻的姿态（蓝色框）
for k in range(0, N+1, 2):  # 为了不看花眼，每隔一步画一次
    R = np.array([[np.cos(Xopt[2, k]), -np.sin(Xopt[2, k])], 
                  [np.sin(Xopt[2, k]), np.cos(Xopt[2, k])]])  # 当前时刻的旋转矩阵
    T = Xopt[0:2, k]  # 当前时刻的位置
    v = np.empty((2, Vv0.shape[1]))
    for j in range(Vv0.shape[1]):
        v[:, j] = R @ Vv0[:, j] + T  # 把原始车身变换到当前位置
    plt.plot(np.append(v[0, :], v[0, 0]), np.append(v[1, :], v[1, 0]), 'b', marker='.') # 画框

# 设置坐标轴比例一致，不然车看着会变形
plt.axis('equal')

# 标签和图例
plt.xlabel('x(m)')
plt.ylabel('y(m)')
plt.gca().tick_params(labelsize=18, width=1.2)
plt.tight_layout()
plt.legend(loc='upper left')

print("显示图像中...")
plt.show()
