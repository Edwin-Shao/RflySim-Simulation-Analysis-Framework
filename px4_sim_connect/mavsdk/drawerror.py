import asyncio
import time
import math
import threading
import UE4CtrlAPI
from mavsdk import System
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 配置
MAV_PORT = 14542
SERVER_PORT = 50058

class DataContext:
    def __init__(self):
        self.ts = []
        self.errs = []
        self.d_att = None # 飞控姿态
        self.u_att = None # UE4姿态
        self.start = time.time()
        self.lock = threading.Lock()
        self.status = "Initializing..." # 用于在图表标题显示状态

    def update_error(self):
        if self.d_att and self.u_att:
            # 计算各轴差值
            diffs = [d - u for d, u in zip(self.d_att, self.u_att)]
            
            # Yaw 角度跳变处理: 限制在 -180 到 180 之间
            for i in range(3):
                if diffs[i] > 180: diffs[i] -= 360
                elif diffs[i] < -180: diffs[i] += 360

            # 均方根误差 (总误差标量)
            total_err = math.sqrt(sum(d**2 for d in diffs))
            
            t = time.time() - self.start
            self.ts.append(t)
            self.errs.append(total_err)

ctx = DataContext()

async def worker():
    try:
        # 1. 连接 MAVSDK
        ctx.status = "Connecting to Drone..."
        drone = System(port=SERVER_PORT)
        await drone.connect(system_address=f"udpin://0.0.0.0:{MAV_PORT}")
        ctx.status = "Drone Connected. Waiting for UE4..."

        # 2. 连接 UE4
        ue = UE4CtrlAPI.UE4CtrlAPI()
        ue.initUE4MsgRec()
        await asyncio.sleep(1)
        ue.reqCamCoptObj(1, [1])
        
        ctx.status = "Running: Capturing Data..."

        async def task_mav():
            await drone.telemetry.set_rate_attitude_euler(25.0)
            async for att in drone.telemetry.attitude_euler():
                with ctx.lock:
                    ctx.d_att = [att.roll_deg, att.pitch_deg, att.yaw_deg]
                    ctx.update_error()

        # 任务: 读取UE4数据
        async def task_ue():
            while True:
                copt = ue.getCamCoptObj(1, 1)
                if copt and copt.hasUpdate:
                    with ctx.lock:
                        ctx.u_att = [math.degrees(x) for x in copt.angEuler]
                        copt.hasUpdate = False
                        ctx.update_error()
                await asyncio.sleep(0.005) 

        await asyncio.gather(task_mav(), task_ue())
    
    except Exception as e:
        ctx.status = f"Error: {str(e)}"

def start_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(worker())

# 主程序
def main():
    # 启动后台线程
    threading.Thread(target=start_loop, daemon=True).start()
    
    # 绘图设置
    fig, ax = plt.subplots(figsize=(10, 6))
    
    line, = ax.plot([], [], 'r-', lw=2, label='Total Error (RMS)')
    
    # 标题用于显示状态
    title = ax.set_title("Status: Starting...", fontsize=12)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Attitude Error (deg)')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')
    
    # 左上角显示数值
    info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, 
                        bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.9))

    def update(_):
        # 从后台获取数据
        with ctx.lock:
            status_str = ctx.status
            if not ctx.ts:
                title.set_text(f"Status: {status_str} (No Data Yet)")
                return line, info_text, title
                
            ts = list(ctx.ts)
            ys = list(ctx.errs)

        # 更新标题状态
        title.set_text(f"Status: {status_str}")

        # 更新曲线
        line.set_data(ts, ys)
        
        # 坐标轴自适应
        if ts:
            ax.set_xlim(0, max(10, ts[-1] + 1))
            ax.set_ylim(0, max(1.0, max(ys) * 1.1)) 

        # 更新平均值文本
        if ys:
            avg_err = sum(ys) / len(ys)
            curr_err = ys[-1]
            info_text.set_text(f"Current: {curr_err:.3f}°\nAverage: {avg_err:.3f}°")
            
        return line, info_text, title

    ani = FuncAnimation(fig, update, interval=100, blit=False)
    plt.show()

if __name__ == "__main__":
    main()