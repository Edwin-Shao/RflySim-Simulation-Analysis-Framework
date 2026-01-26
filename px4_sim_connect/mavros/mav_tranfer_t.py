# 实飞记录：保存 飞控位姿 + UE 位姿 到两个 json，然后用另一个脚本画图
import asyncio
import math
import time
import json
import os
from datetime import datetime

from mavsdk import System
import UE4CtrlAPI as UE4CtrlAPI

LISTEN_PORT = 14541
LOG_DIR = r"D:\code\NIMTE\rflysim\flylog"

UE_REQ_TYPE = 1
UE_TARGET_ID = 1

SAVE_EVERY_S = 5.0
SEND_TO_UE_HZ = 30.0

real_pos = [0.0, 0.0, -0.0]


def now_name():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _get_first_attr(obj, names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    raise AttributeError(f"missing attrs: {names}")


async def run():
    os.makedirs(LOG_DIR, exist_ok=True)

    drone = System()
    print(f"正在监听真机数据 (UDP {LISTEN_PORT})...")
    await drone.connect(system_address=f"udpin://0.0.0.0:{LISTEN_PORT}")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("已连接飞控")
            break

    ue = UE4CtrlAPI.UE4CtrlAPI()

    ue.reqCamCoptObj(UE_REQ_TYPE, [UE_TARGET_ID])
    await asyncio.sleep(1.0)
    ue.initUE4MsgRec()
    await asyncio.sleep(1.0)

    await telemetry_loop(drone, ue)


async def telemetry_loop(drone, ue):
    start_perf = time.perf_counter()

    fc_pos = [real_pos[0], real_pos[1], real_pos[2]]
    fc_att_rad = [0.0, 0.0, 0.0]

    # 两份日志（分别保存）
    fc_log = []
    ue_log = []

    tag = now_name()
    fc_path = os.path.join(LOG_DIR, f"FC_{tag}.json")
    ue_path = os.path.join(LOG_DIR, f"UE_{tag}.json")
    print("开始记录：")
    print(" -", fc_path)
    print(" -", ue_path)

    async def update_position():
        async for odom in drone.telemetry.position_velocity_ned():
            fc_pos[0] = odom.position.north_m + real_pos[0]
            fc_pos[1] = odom.position.east_m + real_pos[1]
            fc_pos[2] = odom.position.down_m + real_pos[2]

    async def update_attitude():
        async for att in drone.telemetry.attitude_euler():
            fc_att_rad[0] = math.radians(att.roll_deg)
            fc_att_rad[1] = math.radians(att.pitch_deg)
            fc_att_rad[2] = math.radians(att.yaw_deg)

    async def keep_alive():
        while True:
            ue.reqCamCoptObj(UE_REQ_TYPE, [UE_TARGET_ID])
            await asyncio.sleep(0.2)

    async def read_ue():
        copt = ue.getCamCoptObj(UE_REQ_TYPE, UE_TARGET_ID)
        if not copt:
            print("[UE] getCamCoptObj returned None（请求没生效/UE没开/ID不对）")
            return

        # 轻量诊断：每秒打印一次 UE 更新次数
        last_print = time.time()
        upd_cnt = 0

        while True:
            if copt.hasUpdate:
                t = time.perf_counter() - start_perf

                pos = _get_first_attr(copt, ["PosUE", "posE", "PosE"])
                ang = _get_first_attr(copt, ["angEuler", "AngEuler", "AngEulerRad"])

                ue_log.append({
                    "t": t,
                    "pos": list(pos),
                    "att_deg": [math.degrees(x) for x in ang],  # 一般是 rad
                })

                copt.hasUpdate = False
                upd_cnt += 1

            now = time.time()
            if now - last_print >= 1.0:
                print(f"[UE] updates/s: {upd_cnt} | total: {len(ue_log)}")
                upd_cnt = 0
                last_print = now

            await asyncio.sleep(0.001)

    # 主循环：持续发送给 UE + 记录飞控日志 + 定期落盘
    last_save = time.time()
    dt_send = 1.0 / SEND_TO_UE_HZ

    asyncio.create_task(update_position())
    asyncio.create_task(update_attitude())
    asyncio.create_task(keep_alive())
    asyncio.create_task(read_ue())

    while True:
        # 发送给 UE（保持你原来的功能）
        ue.sendUE4PosNew(
            copterID=UE_TARGET_ID,
            vehicleType=3,
            PosE=fc_pos,
            AngEuler=fc_att_rad,
            windowID=-1
        )

        # 记录飞控（用同一 perf 基准 t）
        t = time.perf_counter() - start_perf
        fc_log.append({
            "t": t,
            "pos": list(fc_pos),
            "att_deg": [math.degrees(x) for x in fc_att_rad],
        })

        # 定期保存
        now = time.time()
        if now - last_save >= SAVE_EVERY_S:
            with open(fc_path, "w", encoding="utf-8") as f:
                json.dump(fc_log, f, ensure_ascii=False, indent=2)
            with open(ue_path, "w", encoding="utf-8") as f:
                json.dump(ue_log, f, ensure_ascii=False, indent=2)
            print(f"[saved] FC:{len(fc_log)} UE:{len(ue_log)}")
            last_save = now

        await asyncio.sleep(dt_send)


if __name__ == "__main__":
    asyncio.run(run())