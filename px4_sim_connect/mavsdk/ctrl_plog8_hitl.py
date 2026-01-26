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
    """ Does Offboard control using position NED coordinates. """

    
    # 创建一个CopterSim状态获取实例，并监听2s钟，获取当前所有CopterSim列表数据
    req = ReqCopterSim.ReqCopterSim()

    # 下面展示，如何使用本接口，不需要知道目标电脑IP的情况下，能够连上远程电脑的CopterSim
    CopterID=1  # 计划仿真的飞机ID
    TargetPort = 20100 + CopterID*2 -1 # 对应CopterSim的发送端口

    # 获取CopterSim所在电脑的IP地址
    TargetIP = req.getSimIpID(CopterID)
    print('Target IP is: ',TargetIP)

    # 请求CopterSim将UDP模式转换为MAVLink_Full，便于本例程的Mavlink控制
    req.sendReSimUdpMode(CopterID,UDP_mode=2)

    # 向CopterSim发送请求，发送MAVLink数据到本电脑
    req.sendReSimIP(CopterID)

    time.sleep(2)

    print('开始连接Mavsdk')

    # 下面开始用mavsdk来控制飞机
    drone = System() # 创建一个mavsdk实例

    # 等待连接上飞控，这里指定任意IP和20100系列端口，类似udp://0.0.0.0:20101
    await drone.connect(system_address="udp://:"+str(TargetPort))


    print("Waiting for drone to connect...")
    # 等待完全确认连接
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
    
    parameter_a = 5.0
    parameter_b = 3.5
    height_z = -5.0
    circle_times = 30
    total_time = circle_times * 2
    target_x = 0.0
    target_y = 0.0

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