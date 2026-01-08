import asyncio
import sys
import math
import time
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

    while True:
        ue.sendUE4PosNew(
            copterID=1, 
            vehicleType=3, 
            PosE=current_pos, 
            AngEuler=current_att,
            windowID=-1
        )
        
        await asyncio.sleep(0.03)

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