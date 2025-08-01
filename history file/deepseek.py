import numpy as np
import matplotlib.pyplot as plt

class Ship:
    def __init__(self, x, y, psi, v):
        self.x = x
        self.y = y
        self.psi = psi
        self.v = v
    
    def update(self, dpsi, dt):
        self.psi += dpsi * dt
        self.x += self.v * np.cos(self.psi) * dt
        self.y += self.v * np.sin(self.psi) * dt
        return self

class CBDR_Avoidance:
    def __init__(self, safe_theta=0.001, k_nav=0.5, k_beta=1.0, gamma=1.0):
        self.safe_theta = safe_theta  # 安全角直径阈值 (rad)
        self.k_nav = k_nav            # 导航增益
        self.k_beta = k_beta          # 避障增益
        self.gamma = gamma            # 屏障函数陡度
    
    def observe(self, ship, target, target_size):
        """计算方位角和角直径"""
        dx = target.x - ship.x
        dy = target.y - ship.y
        R = np.hypot(dx, dy)
        beta = np.arctan2(dy, dx) - ship.psi
        theta = target_size / R
        return beta, theta
    
    def control(self, ship, target, goal, target_size):
        """航向控制律"""
        beta, theta = self.observe(ship, target, target_size)
        
        # 屏障函数增强
        barrier = 1 + np.exp(-self.gamma * (theta - self.safe_theta/2))
        
        if theta < self.safe_theta:
            # 导航模式：朝向目标
            psi_goal = np.arctan2(goal[1]-ship.y, goal[0]-ship.x)
            dpsi = self.k_nav * np.arctan2(np.sin(psi_goal-ship.psi), 
                                          np.cos(psi_goal-ship.psi))
        else:
            # 避障模式：破坏CBDR
            dpsi = self.k_beta * theta * barrier * np.sign(beta) * np.exp(-np.abs(beta))
        
        return dpsi

# 仿真参数
dt = 0.01
T = 5000
target_size = 10.0  # 目标船尺寸 (m)

# 初始化
own_ship = Ship(x=0, y=0, psi=0, v=3.0)
target_ship = Ship(x=100, y=50, psi=np.pi, v=5.0)  # 直线运动
goal = [200, 150]  # 目标点
controller = CBDR_Avoidance(safe_theta=0.001, k_beta=1.0)

# 记录轨迹
traj_own = []
traj_target = []

for t in range(T):
    # 更新目标船（直线运动）
    target_ship.update(dpsi=0, dt=dt)
    
    # 计算控制指令
    dpsi = controller.control(own_ship, target_ship, goal, target_size)
    
    # 更新自船状态
    own_ship.update(dpsi, dt)
    
    # 保存轨迹
    traj_own.append([own_ship.x, own_ship.y])
    traj_target.append([target_ship.x, target_ship.y])

# 可视化
traj_own = np.array(traj_own)
traj_target = np.array(traj_target)

plt.figure(figsize=(10, 6))
plt.plot(traj_own[:,0], traj_own[:,1], 'b-', label='Own Ship')
plt.plot(traj_target[:,0], traj_target[:,1], 'r--', label='Target Ship')
plt.scatter(goal[0], goal[1], c='g', marker='*', s=200, label='Goal')
plt.xlabel('East (m)'); plt.ylabel('North (m)')
plt.title('CBDR Avoidance with Angular Diameter Control')
plt.legend(); plt.grid(True); plt.axis('equal')
plt.show()