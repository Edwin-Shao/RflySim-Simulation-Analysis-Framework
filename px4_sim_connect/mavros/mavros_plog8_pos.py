import PX4MavCtrlV4ROS as PX4MavCtrl
import time
import math
import socket
import json
import os
from datetime import datetime

TARGET_UDP_IP = "127.0.0.1"
TARGET_UDP_PORT = 16520

LOG_DIR = r"/mnt/d/code/NIMTE/rflysim/flylog"
SAVE_EVERY_S = 5.0

DT = 0.1
k = 0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

mav = PX4MavCtrl.PX4MavCtrler() 
time.sleep(1)

height_z = -1.5

os.makedirs(LOG_DIR, exist_ok=True)
tag = datetime.now().strftime("%Y%m%d_%H%M%S")
fc_path = os.path.join(LOG_DIR, f"FC_{tag}.json")
fc_log = []
_last_save = time.time()


def _maybe_rad_to_deg3(euler3):
    if euler3 is None or len(euler3) < 3:
        return None
    x, y, z = float(euler3[0]), float(euler3[1]), float(euler3[2])
    # 简单判断：都很小一般是弧度
    if max(abs(x), abs(y), abs(z)) < 3.2:
        return [math.degrees(x), math.degrees(y), math.degrees(z)]
    return [x, y, z]


def _flush_fc():
    with open(fc_path, "w", encoding="utf-8") as f:
        json.dump(fc_log, f, ensure_ascii=False, indent=2)
    print(f"[saved] FC:{len(fc_log)} -> {fc_path}")

def generate8(center_n, center_e, radius_a, radius_b, t, period):
    """
    生成八字轨迹点
    :param center_n: 八字中心N坐标
    :param center_e: 八字中心E坐标
    :param radius_a: 八字N方向半径
    :param radius_b: 八字E方向半径
    :param t: 当前时刻（秒）
    :param period: 单圈时间（秒）
    :return: (n, e)
    """
    phase = (t % period) / period * 2 * math.pi
    n = center_n + radius_a * math.sin(phase)
    e = center_e + radius_b * math.cos(phase) * math.sin(phase)
    return n, e

def SendRealPosNED(n, e, d, yaw):
    """
    发送真实位置指令）
    :param n: 北坐标
    :param e: 东坐标
    :param d: 地坐标
    :param yaw: 偏航角
    :return: None
    """
    mav.SendPosNED(e, -n, d, yaw)


print("进入offboard并解锁")

mav.initOffboard()
time.sleep(5)
spos= mav.uavPosNED[:]
local_n = spos[0]
local_e = spos[1]

print("初始位置:", spos)
print("发送起飞命令")
SendRealPosNED(local_n, local_e, height_z + spos[2], 0)
time.sleep(5)

sock.sendto(
    json.dumps({"type": "start", "dt": DT, "ts": time.time()}).encode("utf-8"),
    (TARGET_UDP_IP, TARGET_UDP_PORT)
)

print("PosE", mav.uavPosNED)
print("VelE", mav.uavVelNED)
print("Euler", mav.uavAngEular)
print("Quaternion", mav.uavAngQuatern)
print("Rate", mav.uavAngRate)

time.sleep(1)
print("Start control.")

parameter_a = 3.0
parameter_b = 2.0
circle_times = 30.0
total_time = circle_times * 2.0
center_n = local_n
center_e = local_e

t0 = time.perf_counter()

while (time.perf_counter() - t0) < total_time:
    t_cmd = k * DT

    target_n, target_e = generate8(center_n, center_e, parameter_a, parameter_b, t_cmd, circle_times)
    target_yaw = 0.0

    SendRealPosNED(target_n, target_e, height_z + spos[2], target_yaw)

    ue_pos = mav.uavPosNED

    msg = {
        "type": "step",
        "k": k,
        "dt": DT,
        "t": t_cmd,
        "pos": [ue_pos[0] - local_n, ue_pos[1] - local_e, height_z - spos[2]],
        # "att_deg": _maybe_rad_to_deg3(getattr(mav, "uavAngEular", None)),
        "att_deg": mav.uavAngEular[:],
        "ts": time.time(),
    }
    sock.sendto(json.dumps(msg).encode("utf-8"), (TARGET_UDP_IP, TARGET_UDP_PORT))

    fc_log.append({
        "k": k,
        "dt": DT,
        "t": t_cmd,
        "target": [target_n - local_n, target_e - local_e, height_z, target_yaw],
        "pos": [(mav.uavPosNED[0] - local_n, mav.uavPosNED[1] - local_e, mav.uavPosNED[2] - spos[2])],
        # "att_deg": _maybe_rad_to_deg3(getattr(mav, "uavAngEular", None)),
        "att_deg": mav.uavAngEular[:],

        "ts": time.time(),
    })

    now = time.time()
    if now - _last_save >= SAVE_EVERY_S:
        _flush_fc()
        _last_save = now

    k += 1

    next_t = t0 + k * DT
    sleep_s = next_t - time.perf_counter()
    if sleep_s > 0:
        time.sleep(sleep_s)

SendRealPosNED(local_n, local_e, height_z + spos[2], 0)

_flush_fc()

print("Landing")
mav.land()