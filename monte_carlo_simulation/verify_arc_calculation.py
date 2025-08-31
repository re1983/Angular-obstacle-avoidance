"""
驗證圓弧運動計算的正確性
"""
import numpy as np
import matplotlib.pyplot as plt

def verify_arc_motion(initial_pos, velocity, heading, rate_of_turn, time):
    """驗證圓弧運動軌跡計算"""
    
    print(f"驗證參數:")
    print(f"  初始位置: {initial_pos}")
    print(f"  速度: {velocity} m/s")
    print(f"  初始航向: {heading}°")
    print(f"  轉彎速度: {rate_of_turn}°/s")
    print(f"  飛行時間: {time}s")
    
    # 模擬軌跡
    positions = []
    current_pos = np.array(initial_pos)
    current_heading = heading
    dt = 0.01
    
    for step in range(int(time / dt)):
        positions.append(current_pos.copy())
        
        # 更新位置和航向
        current_heading += rate_of_turn * dt
        current_pos += velocity * dt * np.array([
            np.cos(np.radians(current_heading)),
            np.sin(np.radians(current_heading)),
            0
        ])
    
    positions = np.array(positions)
    final_pos = positions[-1]
    
    print(f"  最終位置: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}]")
    print(f"  最終航向: {current_heading:.1f}°")
    
    # 繪製軌跡
    plt.figure(figsize=(10, 8))
    plt.plot(positions[:, 1], positions[:, 0], 'b-', linewidth=2, label='Ship A軌跡')
    plt.plot(initial_pos[1], initial_pos[0], 'go', markersize=10, label='初始位置')
    plt.plot(final_pos[1], final_pos[0], 'ro', markersize=10, label='最終位置')
    
    # 繪製目標碰撞點
    plt.plot(0, 25, 'rx', markersize=15, label='目標碰撞點')
    
    # 繪製Ownship路徑
    plt.plot([0, 0], [0, 50], 'k--', linewidth=2, alpha=0.7, label='Ownship路徑')
    
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Ship A軌跡驗證')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
    
    return final_pos

if __name__ == "__main__":
    # 測試右轉圓弧
    print("=== 右轉圓弧運動驗證 ===")
    verify_arc_motion([14.23, 15.38, 0.00], 2.0, 180.0, 10.0, 25.0)
    
    # 測試左轉圓弧  
    print("\n=== 左轉圓弧運動驗證 ===")
    verify_arc_motion([14.23, -15.38, 0.00], 2.0, 180.0, -10.0, 25.0)
    
    # 測試直線運動
    print("\n=== 直線運動驗證 ===")
    verify_arc_motion([75.00, 0.00, 0.00], 2.0, 180.0, 0.0, 25.0)
