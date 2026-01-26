import asyncio
import time
import math
import threading
import UE4CtrlAPI
from mavsdk import System
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.start_t = time.time()
        self.t_att = [] 
        self.err_att = []
        self.t_pos = []
        self.err_pos = []
        self.ue_att = None # [r, p, y]
        self.ue_pos = None # [n, e, d]

    # 当收到飞控姿态时调用
    def on_fc_att(self, r, p, y):
        with self.lock:
            if self.ue_att:
                diffs = [r - self.ue_att[0], p - self.ue_att[1], y - self.ue_att[2]]
                diffs = [(d - 360) if d > 180 else (d + 360) if d < -180 else d for d in diffs]
                err = math.sqrt(sum(d**2 for d in diffs))                
                self.t_att.append(time.time() - self.start_t)
                self.err_att.append(err)

    def on_fc_pos(self, n, e, d):
        with self.lock:
            if self.ue_pos:
                diffs = [n - self.ue_pos[0], e - self.ue_pos[1], d - self.ue_pos[2]]
                err = math.sqrt(sum(x**2 for x in diffs))
                
                self.t_pos.append(time.time() - self.start_t)
                self.err_pos.append(err)

data = SharedData()

async def run_backend():
    drone = System(port=50058)
    await drone.connect(system_address="udpin://0.0.0.0:14542")
    ue = UE4CtrlAPI.UE4CtrlAPI()
    ue.initUE4MsgRec()
    
    async def loop_ue():
        while True:
            ue.reqCamCoptObj(1, [1]) 
            for _ in range(5): 
                copt = ue.getCamCoptObj(1, 1)
                if copt and copt.hasUpdate:
                    with data.lock:
                        data.ue_att = [math.degrees(x) for x in copt.angEuler]
                        data.ue_pos = copt.posE
                        copt.hasUpdate = False
            await asyncio.sleep(0.02)

    async def loop_mav_att():
        async for att in drone.telemetry.attitude_euler():
            data.on_fc_att(att.roll_deg, att.pitch_deg, att.yaw_deg)
    async def loop_mav_pos():
        async for odom in drone.telemetry.position_velocity_ned():
            data.on_fc_pos(odom.position.north_m, odom.position.east_m, odom.position.down_m)

    await asyncio.gather(loop_ue(), loop_mav_att(), loop_mav_pos())

def start_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_backend())

def main():
    t = threading.Thread(target=start_thread, daemon=True)
    t.start()
    
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
    fig.canvas.manager.set_window_title('Real-time Error Analysis')
    
    line1, = ax1.plot([], [], 'r-', lw=1.5, label='Attitude RMS (deg)')
    ax1.legend(loc='upper right')
    ax1.grid(True, linestyle='--')
    ax1.set_ylabel('Attitude Error (deg)')
    title = ax1.set_title("Waiting for data...")

    line2, = ax2.plot([], [], 'b-', lw=1.5, label='Position RMS (m)')
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle='--')
    ax2.set_ylabel('Position Error (m)')
    ax2.set_xlabel('Time (s)')

    def update(_):
        with data.lock:
            t_a, y_a = list(data.t_att), list(data.err_att)
            t_p, y_p = list(data.t_pos), list(data.err_pos)
        if not t_a and not t_p: 
            return line1, line2, title
        if t_a:
            line1.set_data(t_a, y_a)
            ax1.set_xlim(0, max(10, t_a[-1] + 1))
            ax1.set_ylim(0, max(0.1, max(y_a) * 1.2))
            title.set_text(f"Attitude Error: {y_a[-1]:.3f}°")
        if t_p:
            line2.set_data(t_p, y_p)
            ax2.set_ylim(0, max(0.1, max(y_p) * 1.2))
        
        return line1, line2, title

    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.show()

if __name__ == "__main__":
    main()