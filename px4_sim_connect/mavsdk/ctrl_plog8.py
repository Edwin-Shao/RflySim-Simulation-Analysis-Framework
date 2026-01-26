#!/usr/bin/env python3

"""
Caveat when attempting to run the examples in non-gps environments:

`drone.offboard.stop()` will return a `COMMAND_DENIED` result because it
requires a mode switch to HOLD, something that is currently not supported in a
non-gps environment.
"""

import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import ReqCopterSim
import time
import math

# mavsdk-python的使用方法见：https://mavsdk.mavlink.io/main/en/python/quickstart.html
# 本平台使用1.2.0版本的mavsdk，因为更高版本无法直接用Python启动，还需运行server
# 接口文档见：http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/

async def run():
    print('开始连接Mavsdk')
    drone = System(port = 50052)
    target_port = 14540
    
    print(f"正在监听 UDP {target_port} 等待连接...")
    await drone.connect(system_address=f"udpin://0.0.0.0:{target_port}")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    # 等待获取健康状态，这里通过查询telemray收集到的数据，请阅读如下文档了解telemray下有哪些数据
    # http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry.html
    async for health in drone.telemetry.health():
        # health()函数见：http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry.html#mavsdk.telemetry.Telemetry.health
        if health.is_global_position_ok and health.is_home_position_ok:
            # health的数据定义见：http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry.html#mavsdk.telemetry.Health
            # 查询得到：is_local_position_ok (bool) – True if the local position estimate is good enough to fly in ‘position control’ mode
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    # 等待解锁飞控
    # 见：http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/action.html
    await drone.action.arm()

    print("-- Setting initial setpoint")
    # http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/offboard.html
    # http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/offboard.html#mavsdk.offboard.Offboard.set_position_ned
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    

    
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(0.5)

    parameter_a = 2.0
    parameter_b = 1.5
    height_z = -2.5
    circle_times = 30
    total_time = circle_times * 2
    target_x = 0.0
    target_y = 0.0

    for _ in range(10):
        await drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, height_z, 0.0))
        await asyncio.sleep(10)

    start_time = time.time()

    while(time.time() - start_time < total_time):
        target_x = parameter_a * math.sin(
            (time.time() - start_time) / circle_times * 2 * math.pi )
        target_y = parameter_b * math.cos(
            (time.time() - start_time) / circle_times * 2 * math.pi ) * math.sin(
                 (time.time() - start_time) / circle_times * 2 * math.pi )
        for _ in range(10):
            await drone.offboard.set_position_ned(
                PositionNedYaw(target_x, target_y, height_z, 0.0))
        await asyncio.sleep(0.1)

    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, height_z, 0.0))
    await asyncio.sleep(5)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
        await drone.action.land()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())