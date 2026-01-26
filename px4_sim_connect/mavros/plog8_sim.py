import PX4MavCtrlV4ROS as PX4MavCtrl
import time
import math
import os
import json
from datetime import datetime
import UE4CtrlAPI as UE4CtrlAPI

mav = PX4MavCtrl.PX4MavCtrler()
time.sleep(1)

LOG_DIR = r"D:\code\NIMTE\rflysim\flylog"
UE_REQ_TYPE = 1
UE_TARGET_ID = 1
SAVE_EVERY_S = 5.0

height_z = -2.5

DT = 0.1


def now_name():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _get_first_attr(obj, names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    raise AttributeError(f"missing attrs: {names}")


def _wsl_winpath_to_mnt(path_str: str) -> str:
    if os.name == "posix" and len(path_str) >= 3 and path_str[1] == ":" and (path_str[2] == "\\" or path_str[2] == "/"):
        drive = path_str[0].lower()
        rest = path_str[2:].replace("\\", "/")
        return f"/mnt/{drive}{rest}"
    return path_str


ue = UE4CtrlAPI.UE4CtrlAPI()
ue.reqCamCoptObj(UE_REQ_TYPE, [UE_TARGET_ID])
time.sleep(1.0)
ue.initUE4MsgRec()
time.sleep(1.0)

copt = ue.getCamCoptObj(UE_REQ_TYPE, UE_TARGET_ID)
if not copt:
    raise RuntimeError("UE getCamCoptObj returned None（请求没生效/UE没开/ID不对）")

_last_req = 0.0


def ue_keep_alive():
    global _last_req
    now = time.time()
    if now - _last_req >= 0.2:
        ue.reqCamCoptObj(UE_REQ_TYPE, [UE_TARGET_ID])
        _last_req = now


log_dir = _wsl_winpath_to_mnt(LOG_DIR)
os.makedirs(log_dir, exist_ok=True)

tag = now_name()
ue_path = os.path.join(log_dir, f"UE_{tag}.json")
print("[UE] logging to:", ue_path)

ue_log = []
_last_save = time.time()

ue_recording = False
start_perf = 0.0

_last_ue_pos = [0.0, 0.0, 0.0]
_last_ue_att_deg = [0.0, 0.0, 0.0]
_has_last_ue = False


def ue_start_recording():
    global ue_recording, start_perf, _last_save, _has_last_ue
    ue_log.clear()
    start_perf = time.perf_counter()
    _last_save = time.time()
    _has_last_ue = False
    ue_recording = True
    print("[UE] recording: ON")


def ue_stop_recording():
    global ue_recording
    ue_recording = False
    print("[UE] recording: OFF")


def ue_poll_once():
    """
    尝试读一帧 UE（如果 hasUpdate=True 就更新缓存并返回 True）
    """
    global _last_ue_pos, _last_ue_att_deg, _has_last_ue

    ue_keep_alive()

    if getattr(copt, "hasUpdate", False):
        pos = _get_first_attr(copt, ["PosUE", "posE", "PosE"])
        ang = _get_first_attr(copt, ["angEuler", "AngEuler", "AngEulerRad"])  # 通常为 rad

        _last_ue_pos = list(pos)
        _last_ue_att_deg = [math.degrees(x) for x in ang]
        _has_last_ue = True

        copt.hasUpdate = False
        return True

    return False


def log_step(k, t_cmd, target_x, target_y, target_z, target_yaw):
    """
    每个控制步写一条：包含指令点 target，用它来对齐两次数据。
    """
    global _last_save

    if not ue_recording:
        return

    updated = ue_poll_once()

    ue_log.append({
        "k": k,                       # 步号：两次运行严格一一对应
        "t": t_cmd,                   # 命令时间轴（k*DT），不是实际 wall-time
        "target": [target_x, target_y, target_z, target_yaw],  # 指令点（用于对齐/算误差）
        "pos": list(_last_ue_pos) if _has_last_ue else None,   # UE 位姿（无则 None）
        "att_deg": list(_last_ue_att_deg) if _has_last_ue else None,
        "ue_valid": bool(_has_last_ue),        # 是否已有 UE 值可用
        "ue_updated": bool(updated),           # 本步是否刚好更新
    })

    now = time.time()
    if now - _last_save >= SAVE_EVERY_S:
        with open(ue_path, "w", encoding="utf-8") as f:
            json.dump(ue_log, f, ensure_ascii=False, indent=2)
        print(f"[saved] UE:{len(ue_log)} -> {ue_path}")
        _last_save = now


def ue_log_flush():
    with open(ue_path, "w", encoding="utf-8") as f:
        json.dump(ue_log, f, ensure_ascii=False, indent=2)
    print(f"[saved-final] UE:{len(ue_log)} -> {ue_path}")

print("进入offboard并解锁")
mav.initOffboard()
time.sleep(5)

print("发送起飞命令")
mav.SendPosNED(0, 0, height_z, 0)
time.sleep(10)

ue_start_recording()

print("PosE", mav.uavPosNED)
print("VelE", mav.uavVelNED)
print("Euler", mav.uavAngEular)
print("Quaternion", mav.uavAngQuatern)
print("Rate", mav.uavAngRate)

time.sleep(1)
print("Start control.")

parameter_a = 2.0
parameter_b = 1.5
circle_times = 30.0
total_time = circle_times * 2.0

total_steps = int(total_time / DT)

t0 = time.perf_counter()

for k in range(total_steps):
    t_cmd = k * DT
    phase = t_cmd / circle_times * 2 * math.pi

    target_n = parameter_a * math.sin(phase)
    target_e = parameter_b * math.cos(phase) * math.sin(phase)
    target_yaw = 0.0

    mav.SendPosNED(target_e, -target_n, height_z, target_yaw)

    log_step(k, t_cmd, target_n, target_e, height_z, target_yaw)

    next_t = t0 + (k + 1) * DT
    sleep_s = next_t - time.perf_counter()
    if sleep_s > 0:
        time.sleep(sleep_s)

mav.SendPosNED(0.0, 0.0, height_z, 0.0)
hold_steps = int(5.0 / DT)
for i in range(hold_steps):
    k = total_steps + i
    t_cmd = k * DT
    log_step(k, t_cmd, 0.0, 0.0, height_z, 0.0)

    next_t = t0 + (k + 1) * DT
    sleep_s = next_t - time.perf_counter()
    if sleep_s > 0:
        time.sleep(sleep_s)

print("Landing")
mav.land()

ue_stop_recording()

ue_log_flush()