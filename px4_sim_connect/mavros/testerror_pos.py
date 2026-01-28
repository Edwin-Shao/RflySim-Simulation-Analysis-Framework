import json
import math
import matplotlib.pyplot as plt

sitl1_JSON = r"D:\code\NIMTE\rflysim\flylog\FC_20260127_sitl1.json"
hitl1_JSON = r"D:\code\NIMTE\rflysim\flylog\FC_20260126_hitl1.json"
real1_JSON = r"D:\code\NIMTE\rflysim\flylog\FC_20260126_lidar1.json"

ROUND_N = 5

def load(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if data and "k" in data[0]:
        data.sort(key=lambda x: x.get("k", 0))
    else:
        data.sort(key=lambda x: x.get("t", 0.0))
    return data

def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0

def att_err_mag_deg(actual_att_deg, target_yaw_deg):
    dr = float(actual_att_deg[0]) - 0.0
    dp = float(actual_att_deg[1]) - 0.0
    dy = wrap_deg(float(actual_att_deg[2]) - float(target_yaw_deg))
    return math.sqrt(dr * dr + dp * dp + dy * dy)

def mean_and_rms(errors):
    if not errors:
        return None, None
    mean_e = sum(errors) / len(errors)
    rms_e = math.sqrt(sum(e * e for e in errors) / len(errors))
    return mean_e, rms_e

def fmt_stat(mean_v, rms_v, nd=3):
    if mean_v is None or rms_v is None:
        return "n/a"
    return f"mean={mean_v:.{nd}f}  rms={rms_v:.{nd}f}"

def set_axes_equal_3d(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = (x_limits[1] + x_limits[0]) / 2.0
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = (y_limits[1] + y_limits[0]) / 2.0
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = (z_limits[1] + z_limits[0]) / 2.0
    plot_radius = 0.5 * max(x_range, y_range, z_range)
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def get_xyz(pos):
    if isinstance(pos, list) and len(pos) > 0 and isinstance(pos[0], list):
        return pos[0]
    return pos

def rel_sim(val, minv, maxv):
    if maxv == minv:
        return 100.0
    return (1 - (val - minv) / (maxv - minv)) * 100

def main():
    sitl1 = load(sitl1_JSON)
    hitl1 = load(hitl1_JSON)
    real1 = load(real1_JSON)

    # 按k索引
    hitl1_by_k = {r['k']: r for r in hitl1 if 'k' in r}
    real1_by_k = {r['k']: r for r in real1 if 'k' in r}

    matched = []
    miss_hitl = 0
    miss_real = 0
    miss_any = 0

    for r_sitl in sitl1:
        k = r_sitl.get('k', None)
        if k is None:
            continue
        r_hitl = hitl1_by_k.get(k, None)
        r_real = real1_by_k.get(k, None)
        if r_hitl is None:
            miss_hitl += 1
            miss_any += 1
            continue
        if r_real is None:
            miss_real += 1
            miss_any += 1
            continue
        matched.append((r_sitl, r_hitl, r_real))

    print(f"sitl1 entries: {len(sitl1)}")
    print(f"hitl1 entries: {len(hitl1)}")
    print(f"real1 entries: {len(real1)}")
    print(f"matched (by k): {len(matched)}")
    print(f"missing in hitl1 (wrt sitl k): {miss_hitl}")
    print(f"missing in real1 (wrt sitl k): {miss_real}")
    print(f"missing in any (skipped): {miss_any}")

    # 轨迹数据
    tx, ty, tz = [], [], []
    sx, sy, sz = [], [], []  # sitl
    hx, hy, hz = [], [], []  # hitl
    rx, ry, rz = [], [], []  # real

    # 误差数据（按 k）
    ks = []

    # 位置误差（模长）
    pos_e_s, pos_e_h, pos_e_r = [], [], []

    # 位置分量误差（pos - target）
    s_dx, s_dy, s_dz = [], [], []
    h_dx, h_dy, h_dz = [], [], []
    r_dx, r_dy, r_dz = [], [], []

    # 姿态误差
    att_e_s, att_e_h, att_e_r = [], [], []

    for i, (rs, rh, rr) in enumerate(matched):
        tgt = rs.get("target", None)
        if not tgt or len(tgt) < 4:
            continue
        tgt_pos = [round(float(tgt[0]), ROUND_N), round(float(tgt[1]), ROUND_N), round(float(tgt[2]), ROUND_N)]
        tgt_yaw = float(tgt[3])

        ps = get_xyz(rs.get("pos", None))
        ph = get_xyz(rh.get("pos", None))
        pr = get_xyz(rr.get("pos", None))

        as_ = rs.get("att_deg", None)
        ah_ = rh.get("att_deg", None)
        ar_ = rr.get("att_deg", None)

        if (
            ps is None or ph is None or pr is None
            or not isinstance(ps, (list, tuple)) or len(ps) < 3
            or not isinstance(ph, (list, tuple)) or len(ph) < 3
            or not isinstance(pr, (list, tuple)) or len(pr) < 3
        ):
            continue

        # 轨迹
        tx.append(tgt_pos[0]); ty.append(tgt_pos[1]); tz.append(tgt_pos[2])
        sx.append(ps[0]); sy.append(ps[1]); sz.append(ps[2])
        hx.append(ph[0]); hy.append(ph[1]); hz.append(ph[2])
        rx.append(pr[0]); ry.append(pr[1]); rz.append(pr[2])

        k_plot = rs.get("k", i)
        ks.append(k_plot)

        # 分量误差
        dsx = float(ps[0]) - tgt_pos[0]
        dsy = float(ps[1]) - tgt_pos[1]
        dsz = float(ps[2]) - tgt_pos[2]
        dhx = float(ph[0]) - tgt_pos[0]
        dhy = float(ph[1]) - tgt_pos[1]
        dhz = float(ph[2]) - tgt_pos[2]
        drx = float(pr[0]) - tgt_pos[0]
        dry = float(pr[1]) - tgt_pos[1]
        drz = float(pr[2]) - tgt_pos[2]

        s_dx.append(dsx); s_dy.append(dsy); s_dz.append(dsz)
        h_dx.append(dhx); h_dy.append(dhy); h_dz.append(dhz)
        r_dx.append(drx); r_dy.append(dry); r_dz.append(drz)

        # 模长误差
        pos_e_s.append(math.sqrt(dsx * dsx + dsy * dsy + dsz * dsz))
        pos_e_h.append(math.sqrt(dhx * dhx + dhy * dhy + dhz * dhz))
        pos_e_r.append(math.sqrt(drx * drx + dry * dry + drz * drz))

        # 姿态误差
        if as_ is None:
            att_e_s.append(None)
        else:
            att_e_s.append(att_err_mag_deg(as_, tgt_yaw))

        if ah_ is None:
            att_e_h.append(None)
        else:
            att_e_h.append(att_err_mag_deg(ah_, tgt_yaw))

        if ar_ is None:
            att_e_r.append(None)
        else:
            att_e_r.append(att_err_mag_deg(ar_, tgt_yaw))

    # 统计
    pos_s_mean, pos_s_rms = mean_and_rms(pos_e_s)
    pos_h_mean, pos_h_rms = mean_and_rms(pos_e_h)
    pos_r_mean, pos_r_rms = mean_and_rms(pos_e_r)

    att_s_valid = [e for e in att_e_s if e is not None]
    att_h_valid = [e for e in att_e_h if e is not None]
    att_r_valid = [e for e in att_e_r if e is not None]
    att_s_mean, att_s_rms = mean_and_rms(att_s_valid)
    att_h_mean, att_h_rms = mean_and_rms(att_h_valid)
    att_r_mean, att_r_rms = mean_and_rms(att_r_valid)

    sdx_mean, sdx_rms = mean_and_rms(s_dx)
    sdy_mean, sdy_rms = mean_and_rms(s_dy)
    sdz_mean, sdz_rms = mean_and_rms(s_dz)
    hdx_mean, hdx_rms = mean_and_rms(h_dx)
    hdy_mean, hdy_rms = mean_and_rms(h_dy)
    hdz_mean, hdz_rms = mean_and_rms(h_dz)
    rdx_mean, rdx_rms = mean_and_rms(r_dx)
    rdy_mean, rdy_rms = mean_and_rms(r_dy)
    rdz_mean, rdz_rms = mean_and_rms(r_dz)

    print(f"sitl pos  {fmt_stat(pos_s_mean, pos_s_rms, nd=4)}")
    print(f"hitl pos  {fmt_stat(pos_h_mean, pos_h_rms, nd=4)}")
    print(f"real pos  {fmt_stat(pos_r_mean, pos_r_rms, nd=4)}")
    if att_s_valid and att_h_valid and att_r_valid:
        print(f"sitl att  {fmt_stat(att_s_mean, att_s_rms, nd=4)}")
        print(f"hitl att  {fmt_stat(att_h_mean, att_h_rms, nd=4)}")
        print(f"real att  {fmt_stat(att_r_mean, att_r_rms, nd=4)}")
    else:
        print("attitude: some entries missing (att_deg is None)")

    # 总位置误差
    all_rms = [pos_s_rms, pos_h_rms, pos_r_rms]
    min_rms = min(all_rms)
    max_rms = max(all_rms)
    sitl_sim = rel_sim(pos_s_rms, min_rms, max_rms)
    hitl_sim = rel_sim(pos_h_rms, min_rms, max_rms)
    real_sim = rel_sim(pos_r_rms, min_rms, max_rms)

    # x分量
    all_x = [sdx_rms, hdx_rms, rdx_rms]
    min_x = min(all_x)
    max_x = max(all_x)
    sdx_sim = rel_sim(sdx_rms, min_x, max_x)
    hdx_sim = rel_sim(hdx_rms, min_x, max_x)
    rdx_sim = rel_sim(rdx_rms, min_x, max_x)

    # y分量
    all_y = [sdy_rms, hdy_rms, rdy_rms]
    min_y = min(all_y)
    max_y = max(all_y)
    sdy_sim = rel_sim(sdy_rms, min_y, max_y)
    hdy_sim = rel_sim(hdy_rms, min_y, max_y)
    rdy_sim = rel_sim(rdy_rms, min_y, max_y)

    # z分量
    all_z = [sdz_rms, hdz_rms, rdz_rms]
    min_z = min(all_z)
    max_z = max(all_z)
    sdz_sim = rel_sim(sdz_rms, min_z, max_z)
    hdz_sim = rel_sim(hdz_rms, min_z, max_z)
    rdz_sim = rel_sim(rdz_rms, min_z, max_z)

    # 姿态
    all_att = [att_s_rms, att_h_rms, att_r_rms]
    min_att = min(all_att)
    max_att = max(all_att)
    att_s_sim = rel_sim(att_s_rms, min_att, max_att)
    att_h_sim = rel_sim(att_h_rms, min_att, max_att)
    att_r_sim = rel_sim(att_r_rms, min_att, max_att)

    # 打印
    print(f"sitl1 归一化相似度: {sitl_sim:.2f}%")
    print(f"hitl1 归一化相似度: {hitl_sim:.2f}%")
    print(f"real1 归一化相似度: {real_sim:.2f}%")
    print(f"x分量归一化相似度: sitl1 {sdx_sim:.2f}%  hitl1 {hdx_sim:.2f}%  real1 {rdx_sim:.2f}%")
    print(f"y分量归一化相似度: sitl1 {sdy_sim:.2f}%  hitl1 {hdy_sim:.2f}%  real1 {rdy_sim:.2f}%")
    print(f"z分量归一化相似度: sitl1 {sdz_sim:.2f}%  hitl1 {hdz_sim:.2f}%  real1 {rdz_sim:.2f}%")
    print(f"姿态归一化相似度: sitl1 {att_s_sim:.2f}%  hitl1 {att_h_sim:.2f}%  real1 {att_r_sim:.2f}%")

    # ---------------- 图1：3D + 模长误差 + 姿态误差 ----------------
    fig = plt.figure(figsize=(12, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[3, 2])

    ax_traj = fig.add_subplot(gs[0, :], projection="3d")
    ax_pos = fig.add_subplot(gs[1, 0])
    ax_att = fig.add_subplot(gs[1, 1])

    ax_traj.plot(tx, ty, tz, "k--", linewidth=1.2, label="target")
    ax_traj.plot(sx, sy, sz, "b-", linewidth=1.2, label="sitl1")
    ax_traj.plot(hx, hy, hz, "r-", linewidth=1.2, label="hitl1")
    ax_traj.plot(rx, ry, rz, "g-", linewidth=1.2, label="real1")
    ax_traj.set_title("3D Trajectory")
    ax_traj.set_xlabel("X (N)")
    ax_traj.set_ylabel("Y (E)")
    ax_traj.set_zlabel("Z (D)")
    ax_traj.legend(loc="upper right")
    ax_traj.grid(True, linestyle="--", alpha=0.4)
    set_axes_equal_3d(ax_traj)

    ax_pos.plot(ks, pos_e_s, "b-", linewidth=0.8, label="sitl1 pos err")
    ax_pos.plot(ks, pos_e_h, "r-", linewidth=0.8, label="hitl1 pos err")
    ax_pos.plot(ks, pos_e_r, "g-", linewidth=0.8, label="real1 pos err")
    ax_pos.set_title("Position error magnitude (m)")
    ax_pos.set_xlabel("k (step)")
    ax_pos.set_ylabel("error (m)")
    ax_pos.grid(True, linestyle="--", alpha=0.6)
    ax_pos.legend(loc="upper right")
    ax_pos.text(
        0.02, 0.98,
        "sitl1 " + fmt_stat(pos_s_mean, pos_s_rms, nd=3) + f"  sim={sitl_sim:.1f}%\n"
        "hitl1 " + fmt_stat(pos_h_mean, pos_h_rms, nd=3) + f"  sim={hitl_sim:.1f}%\n"
        "real1 " + fmt_stat(pos_r_mean, pos_r_rms, nd=3) + f"  sim={real_sim:.1f}%",
        transform=ax_pos.transAxes,
        va="top", ha="left",
        bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.9)
    )

    ks_as = [k for k, e in zip(ks, att_e_s) if e is not None]
    ks_ah = [k for k, e in zip(ks, att_e_h) if e is not None]
    ks_ar = [k for k, e in zip(ks, att_e_r) if e is not None]
    as_plot = [e for e in att_e_s if e is not None]
    ah_plot = [e for e in att_e_h if e is not None]
    ar_plot = [e for e in att_e_r if e is not None]

    ax_att.plot(ks_as, as_plot, "b-", linewidth=1.2, label="sitl1 att err")
    ax_att.plot(ks_ah, ah_plot, "r-", linewidth=1.2, label="hitl1 att err")
    ax_att.plot(ks_ar, ar_plot, "g-", linewidth=1.2, label="real1 att err")
    ax_att.set_title("Attitude error (deg)")
    ax_att.set_xlabel("k (step)")
    ax_att.set_ylabel("error (deg)")
    ax_att.grid(True, linestyle="--", alpha=0.6)
    ax_att.legend(loc="upper right")
    if att_s_valid and att_h_valid and att_r_valid:
        ax_att.text(
            0.02, 0.98,
            "sitl1 " + fmt_stat(att_s_mean, att_s_rms, nd=3) + f"  sim={att_s_sim:.1f}%\n"
            "hitl1 " + fmt_stat(att_h_mean, att_h_rms, nd=3) + f"  sim={att_h_sim:.1f}%\n"
            "real1 " + fmt_stat(att_r_mean, att_r_rms, nd=3) + f"  sim={att_r_sim:.1f}%",
            transform=ax_att.transAxes,
            va="top", ha="left",
            bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.9)
        )

    fig.tight_layout()

    # ---------------- 图2：位置 xyz 分量误差 ----------------
    fig_xyz, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig_xyz.suptitle("Position component error (m): pos - target")

    def _style_axis(ax, title, ys, yh, yr, st_s, st_h, st_r, sim_s, sim_h, sim_r):
        ax.plot(ks, ys, "b-", linewidth=1.1, label=f"sitl1 {title}")
        ax.plot(ks, yh, "r-", linewidth=1.1, label=f"hitl1 {title}")
        ax.plot(ks, yr, "g-", linewidth=1.1, label=f"real1 {title}")
        ax.axhline(0.0, color="k", linewidth=0.8, alpha=0.5)
        ax.set_ylabel("error (m)")
        ax.set_title(title)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(loc="upper right")
        ax.text(
            0.01, 0.96,
            f"sitl1 {st_s}  sim={sim_s:.1f}%\n"
            f"hitl1 {st_h}  sim={sim_h:.1f}%\n"
            f"real1 {st_r}  sim={sim_r:.1f}%",
            transform=ax.transAxes,
            va="top", ha="left",
            bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.9)
        )

    _style_axis(
        ax_x, "dx = x - x_target",
        s_dx, h_dx, r_dx,
        fmt_stat(sdx_mean, sdx_rms, nd=3),
        fmt_stat(hdx_mean, hdx_rms, nd=3),
        fmt_stat(rdx_mean, rdx_rms, nd=3),
        sdx_sim, hdx_sim, rdx_sim
    )
    _style_axis(
        ax_y, "dy = y - y_target",
        s_dy, h_dy, r_dy,
        fmt_stat(sdy_mean, sdy_rms, nd=3),
        fmt_stat(hdy_mean, hdy_rms, nd=3),
        fmt_stat(rdy_mean, rdy_rms, nd=3),
        sdy_sim, hdy_sim, rdy_sim
    )
    _style_axis(
        ax_z, "dz = z - z_target",
        s_dz, h_dz, r_dz,
        fmt_stat(sdz_mean, sdz_rms, nd=3),
        fmt_stat(hdz_mean, hdz_rms, nd=3),
        fmt_stat(rdz_mean, rdz_rms, nd=3),
        sdz_sim, hdz_sim, rdz_sim
    )

    ax_z.set_xlabel("k (step)")
    fig_xyz.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()