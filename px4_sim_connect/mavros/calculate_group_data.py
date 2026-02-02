import json
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

group_idx = 1
sitl1_JSON = rf"D:\code\NIMTE\rflysim\flylog\FC_20260129_sitl{group_idx}.json"
hitl1_JSON = rf"D:\code\NIMTE\rflysim\flylog\FC_20260129_hitl{group_idx}.json"
real1_JSON = rf"D:\code\NIMTE\rflysim\flylog\FC_20260129_lidar{group_idx}.json"

def save_to_excel(data_dict, filename, group_idx):
    """
    data_dict: {"SITL pos x": [...], "SITL pos y": [...], ...}
    filename: excel 文件名
    group_idx: 当前是第几组数据（列号）
    """
    try:
        df = pd.read_excel(filename, index_col=0)
    except FileNotFoundError:
        df = pd.DataFrame()
    for key, vals in data_dict.items():
        col_name = f"{key}_group{group_idx}"
        df[col_name] = pd.Series(vals)
    df.to_excel(filename)

def load(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if data and "k" in data[0]:
        data.sort(key=lambda x: x.get("k", 0))
    else:
        data.sort(key=lambda x: x.get("t", 0.0))
    return data

def norm3(v):
    flat = []
    for x in v:
        if isinstance(x, list):
            flat.extend(x)
        else:
            flat.append(x)
    return math.sqrt(sum(float(x)**2 for x in flat if isinstance(x, (int, float, str))))

def rms(errors):
    if not errors:
        return 0.0
    return math.sqrt(sum(e**2 for e in errors) / len(errors))

def comp_percent(errors, rms, par=2):
    return sum(abs(e) < par * rms for e in errors) / len(errors) * 100 if errors else 0.0

def get_first_number(x):
    if isinstance(x, list):
        return get_first_number(x[0])
    return x

def pad3(lst, fill=0):
    lst = lst if isinstance(lst, list) else [lst]
    flat = []
    for x in lst:
        if isinstance(x, list):
            flat.extend(x)
        else:
            flat.append(x)
    return (flat + [fill, fill, fill])[:3]

def main():
    sitl1 = load(sitl1_JSON)
    hitl1 = load(hitl1_JSON)
    real1 = load(real1_JSON)

    sitl_by_k = {r['k']: r for r in sitl1 if 'k' in r}
    hitl_by_k = {r['k']: r for r in hitl1 if 'k' in r}
    real_by_k = {r['k']: r for r in real1 if 'k' in r}

    ks = sorted(set(sitl_by_k.keys()) & set(hitl_by_k.keys()) & set(real_by_k.keys()))

    par = 2

    sitl_pos_diffs = [[], [], []]
    sitl_att_diffs = [[], [], []]
    hitl_pos_diffs = [[], [], []]
    hitl_att_diffs = [[], [], []]

    sitl_pos_norm_diffs = []
    sitl_att_norm_diffs = []
    hitl_pos_norm_diffs = []
    hitl_att_norm_diffs = []

    for k in ks:
        s = sitl_by_k[k]
        h = hitl_by_k[k]
        r = real_by_k[k]
        s_pos = pad3(s.get("pos", [0,0,0]))
        h_pos = pad3(h.get("pos", [0,0,0]))
        r_pos = pad3(r.get("pos", [0,0,0]))
        s_att = pad3(s.get("att_deg", [0,0,0]))
        h_att = pad3(h.get("att_deg", [0,0,0]))
        r_att = pad3(r.get("att_deg", [0,0,0]))
        for i in range(3):
            s_val = get_first_number(s_pos[i])
            r_val = get_first_number(r_pos[i])
            h_val = get_first_number(h_pos[i])
            sitl_pos_diffs[i].append(s_val - r_val)
            hitl_pos_diffs[i].append(h_val - r_val)
            s_att_val = get_first_number(s_att[i])
            r_att_val = get_first_number(r_att[i])
            h_att_val = get_first_number(h_att[i])
            sitl_att_diffs[i].append(s_att_val - r_att_val)
            hitl_att_diffs[i].append(h_att_val - r_att_val)
        sitl_pos_norm_diffs.append(norm3(s_pos) - norm3(r_pos))
        sitl_att_norm_diffs.append(norm3(s_att) - norm3(r_att))
        hitl_pos_norm_diffs.append(norm3(h_pos) - norm3(r_pos))
        hitl_att_norm_diffs.append(norm3(h_att) - norm3(r_att))

    for name, diffs in [
        ("SITL pos", sitl_pos_diffs),
        ("SITL att", sitl_att_diffs),
        ("HITL pos", hitl_pos_diffs),
        ("HITL att", hitl_att_diffs)
    ]:
        axes_name = ['x', 'y', 'z'] if 'pos' in name else ['roll', 'pitch', 'yaw']
        fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        for i, axis in enumerate(axes_name):
            vals = diffs[i]
            rms_val = rms(vals)
            percent = comp_percent(vals, rms_val, par)
            unit = "m" if 'pos' in name else "deg"
            mean_val = np.mean(vals)
            max_val = np.max(vals)
            min_val = np.min(vals)
            print(f"{name} {axis}: RMS={rms_val:.4f}({unit}), mean={mean_val:.4f}, max={max_val:.4f}, min={min_val:.4f}, percent |error| < {par}*RMS: {percent:.2f}%")

            k_idx = np.arange(len(vals))
            axes[i].plot(k_idx, vals, 'o', label="Error")
            axes[i].axhline(par*rms_val, color='r', linestyle='--', label=f"+{par}×RMS" if i==0 else None)
            axes[i].axhline(-par*rms_val, color='r', linestyle='--')
            axes[i].axhline(0, color='k', linestyle='-', linewidth=0.8)
            axes[i].set_ylabel(f"{axis} Error")
            axes[i].set_title(f"{name} {axis} error vs k")
            axes[i].grid(True, linestyle="--", alpha=0.5)
            if i == 0:
                axes[i].legend()
        axes[-1].set_xlabel("k (step index)")
        plt.tight_layout()
        plt.show()

    for name, diffs, unit in [
        ("SITL pos norm", sitl_pos_norm_diffs, "m"),
        ("SITL att norm", sitl_att_norm_diffs, "deg"),
        ("HITL pos norm", hitl_pos_norm_diffs, "m"),
        ("HITL att norm", hitl_att_norm_diffs, "deg"),
    ]:
        rms_val = rms(diffs)
        percent = comp_percent(diffs, rms_val, par)
        print(f"{name}: RMS={rms_val:.4f}({unit}), percent |error| < {par}*RMS: {percent:.2f}%")

    rms_names, rms_values = [], []
    percent_names, percent_values = [], []

    stat_items = [
        ("SITL pos x", sitl_pos_diffs[0]),
        ("SITL pos y", sitl_pos_diffs[1]),
        ("SITL pos z", sitl_pos_diffs[2]),
        ("SITL att roll", sitl_att_diffs[0]),
        ("SITL att pitch", sitl_att_diffs[1]),
        ("SITL att yaw", sitl_att_diffs[2]),
        ("HITL pos x", hitl_pos_diffs[0]),
        ("HITL pos y", hitl_pos_diffs[1]),
        ("HITL pos z", hitl_pos_diffs[2]),
        ("HITL att roll", hitl_att_diffs[0]),
        ("HITL att pitch", hitl_att_diffs[1]),
        ("HITL att yaw", hitl_att_diffs[2]),
    ]
    for name, diffs in stat_items:
        rms_val = rms(diffs)
        percent = comp_percent(diffs, rms_val, par)
        rms_names.append(f"{name} RMS")
        percent_names.append(f"{name} percent")
        rms_values.append(rms_val)
        percent_values.append(percent)

    norm_items = [
        ("SITL pos norm", sitl_pos_norm_diffs),
        ("SITL att norm", sitl_att_norm_diffs),
        ("HITL pos norm", hitl_pos_norm_diffs),
        ("HITL att norm", hitl_att_norm_diffs),
    ]
    for name, diffs in norm_items:
        rms_val = rms(diffs)
        percent = comp_percent(diffs, rms_val, par)
        rms_names.append(f"{name} RMS")
        percent_names.append(f"{name} percent")
        rms_values.append(rms_val)
        percent_values.append(percent)

    stat_names = rms_names + percent_names
    stat_values = rms_values + percent_values

    # 保存到Excel
    import os
    excel_file = "result.xlsx"
    
    col_idx = group_idx + 1  # C列为第一组，D列为第二组...

    if os.path.exists(excel_file):
        df = pd.read_excel(excel_file, index_col=0)
    else:
        df = pd.DataFrame(index=stat_names)
        df.index.name = "统计项"
    df = df.reindex(stat_names)
    while df.shape[1] < col_idx:
        df.insert(df.shape[1], f"group{df.shape[1]}", "")
    df.iloc[:, col_idx-1] = stat_values
    df.to_excel(excel_file)

if __name__ == "__main__":
    main()