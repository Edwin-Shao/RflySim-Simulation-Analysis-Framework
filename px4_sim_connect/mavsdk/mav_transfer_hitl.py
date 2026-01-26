"这个是硬件在环的版本，主要是为了记录真实飞行器的位姿数据，同时将数据传输给UE4进行仿真显示"
import asyncio
import sys
import math
import time
import json
import os
from datetime import datetime
from mavsdk import System
import UE4CtrlAPI as UE4CtrlAPI


LISTEN_PORT = 14541 

real_pos = [-2.199, 1.095, -7.802]


async def run():

    ue = UE4CtrlAPI.UE4CtrlAPI()

    drone = System()
    print(f"正在监听真机数据 (UDP {LISTEN_PORT})...")
    await drone.connect(system_address=f"udpin://0.0.0.0:{LISTEN_PORT}")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("已连接")
            break

    await asyncio.gather(
        telemetry_loop(drone, ue)
    )

async def telemetry_loop(drone, ue):
    
    current_pos = [real_pos[0], real_pos[1], real_pos[2]]
    current_att = [0, 0, 0] 

    asyncio.create_task(update_position(drone, current_pos))
    asyncio.create_task(update_attitude(drone, current_att))

    log_data = []
    filename = f"FullLog_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    last_save_time = time.time()
    save_dir = os.path.join(r"D:\code\MIMTE\rflysim\flylog", filename)
    print(f"准备记录位姿数据")

    while True:
        record_pos = [current_pos[0] - real_pos[0], current_pos[1] - real_pos[1], current_pos[2] - real_pos[2]]
        now = time.time()
        log_data.append({
            "timestamp": now,
            "time_str": datetime.now().strftime('%H:%M:%S.%f'),
            "position": list(record_pos),
            "attitude": list(current_att)
        })

        if now - last_save_time > 5.0:
            with open(save_dir, 'w') as f:
                json.dump(log_data, f, indent=4)
                last_save_time = now

        ue.sendUE4PosNew(
            copterID=1, 
            vehicleType=3, 
            PosE=current_pos, 
            AngEuler=current_att,
            windowID=-1
        )

        ue_data = ue.getUE4Pos(1)
        
        if ue_data[3] == 1:
            ue_pos_x = ue_data[0]
            ue_pos_y = ue_data[1]
            ue_pos_z = ue_data[2]
            
            # 计算指令与实际的距离
            err_x = current_pos[0] - ue_pos_x
            err_y = current_pos[1] - ue_pos_y
            err_z = current_pos[2] - ue_pos_z
            
            total_error = math.sqrt(err_x**2 + err_y**2 + err_z**2)
            
            if total_error > 0.1:
                print(f"误差为：{total_error:.2f}m")

        await asyncio.sleep(1/30)

async def update_position(drone, pos_ref):
    async for odom in drone.telemetry.position_velocity_ned():
        pos_ref[0] = odom.position.north_m + real_pos[0]
        pos_ref[1] = odom.position.east_m + real_pos[1]
        pos_ref[2] = odom.position.down_m + real_pos[2]

async def update_attitude(drone, att_ref):
    async for att in drone.telemetry.attitude_euler():
        att_ref[0] = math.radians(att.roll_deg)
        att_ref[1] = math.radians(att.pitch_deg)
        att_ref[2] = math.radians(att.yaw_deg)

if __name__ == "__main__":
    asyncio.run(run())