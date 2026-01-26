import json, math
import matplotlib.pyplot as plt

FC_JSON = r"D:\code\NIMTE\rflysim\flylog\UE_20260120_101742.json"
UE_JSON = r"D:\code\NIMTE\rflysim\flylog\UE_20260120_102041.json"

MAX_DT = 1
USE_INTERP = False

T_START = 3.0
T_END   = None   # 无限制： None

def wrap_deg(d):
    if d > 180: d -= 360
    elif d < -180: d += 360
    return d

def interp_angle(a0, a1, w):
    return wrap_deg(a0 + w * wrap_deg(a1 - a0))

def load(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    data.sort(key=lambda x: x["t"])
    return data

def match_by_time(data, t, use_interp):
    """
    在 data(按 t 排序) 中，用二分找最接近 t 的样本。
    - use_interp=False: 最近邻
    - use_interp=True : 用左右两帧线性插值到 t（yaw 做环绕插值）
    返回 (dt, sample) 或 None
    """
    if not data:
        return None

    lo, hi = 0, len(data)
    while lo < hi:
        mid = (lo + hi) // 2
        if data[mid]["t"] < t:
            lo = mid + 1
        else:
            hi = mid
    j = lo

    if j == 0:
        dt = abs(data[0]["t"] - t)
        return (dt, data[0]) if dt <= MAX_DT else None
    if j >= len(data):
        dt = abs(data[-1]["t"] - t)
        return (dt, data[-1]) if dt <= MAX_DT else None

    a = data[j - 1]
    b = data[j]
    t0, t1 = a["t"], b["t"]

    if (not use_interp) or (t1 <= t0):
        da = abs(t - t0)
        db = abs(t1 - t)
        pick = a if da <= db else b
        dt = min(da, db)
        return (dt, pick) if dt <= MAX_DT else None

    w = (t - t0) / (t1 - t0)
    dt = max(abs(t - t0), abs(t1 - t))
    if dt > MAX_DT:
        return None

    p0, p1 = a["pos"], b["pos"]
    r0, r1 = a["att_deg"], b["att_deg"]

    pos = [p0[i] + w * (p1[i] - p0[i]) for i in range(3)]
    att = [
        r0[0] + w * (r1[0] - r0[0]),
        r0[1] + w * (r1[1] - r0[1]),
        interp_angle(r0[2], r1[2], w),
    ]
    return dt, {"t": t, "pos": pos, "att_deg": att}

def main():
    fc = load(FC_JSON)
    ue = load(UE_JSON)

    ts = []
    pos_rms = []
    att_rms = []
    dts_ms = []

    for u in ue:
        t = u["t"]
        if T_START is not None and t < T_START:
            continue
        if T_END is not None and t > T_END:
            break
        m = match_by_time(fc, t, USE_INTERP)
        if not m:
            continue
        dt, f = m

        fp = f["pos"]; up = u["pos"]
        fa = f["att_deg"]; ua = u["att_deg"]

        dx = fp[0] - up[0]
        dy = fp[1] - up[1]
        dz = fp[2] - up[2]
        perr = math.sqrt(dx*dx + dy*dy + dz*dz)

        dr = fa[0] - ua[0]
        dp = fa[1] - ua[1]
        dyaw = wrap_deg(fa[2] - ua[2])
        aerr = math.sqrt(dr*dr + dp*dp + dyaw*dyaw)

        ts.append(t)
        pos_rms.append(perr)
        att_rms.append(aerr)
        dts_ms.append(dt * 1000.0)

    print(f"matched: {len(ts)} / {len(ue)}")
    if dts_ms:
        print(f"dt avg(ms): {sum(dts_ms)/len(dts_ms):.2f}  max(ms): {max(dts_ms):.2f}")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax1.plot(ts, pos_rms, "b-")
    ax1.set_ylabel("Position RMS (m)")
    ax1.grid(True, linestyle="--", alpha=0.6)

    ax2.plot(ts, att_rms, "r-")
    ax2.set_ylabel("Attitude RMS (deg)")
    ax2.set_xlabel("t (s, perf)")
    ax2.grid(True, linestyle="--", alpha=0.6)

    fig.suptitle(f"UE->FC match  max_dt={MAX_DT*1000:.1f}ms  interp={USE_INTERP}")
    plt.show()

if __name__ == "__main__":
    main()