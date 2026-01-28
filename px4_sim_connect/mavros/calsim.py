import json
import math
import matplotlib.pyplot as plt

sitl1_JSON = r"D:\code\NIMTE\rflysim\flylog\FC_20260127_sitl1.json"
hitl1_JSON = r"D:\code\NIMTE\rflysim\flylog\FC_20260126_hitl1.json"
real1_JSON = r"D:\code\NIMTE\rflysim\flylog\FC_20260126_lidar1" \
".json"

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
        sitl_pos_norm_diffs.append(abs(norm3(s_pos) - norm3(r_pos)))
        sitl_att_norm_diffs.append(abs(norm3(s_att) - norm3(r_att)))
        hitl_pos_norm_diffs.append(abs(norm3(h_pos) - norm3(r_pos)))
        hitl_att_norm_diffs.append(abs(norm3(h_att) - norm3(r_att)))

    for name, diffs in [
        ("SITL pos", sitl_pos_diffs),
        ("SITL att", sitl_att_diffs),
        ("HITL pos", hitl_pos_diffs),
        ("HITL att", hitl_att_diffs)
    ]:
        for i, axis in enumerate(['x', 'y', 'z'] if 'pos' in name else ['roll', 'pitch', 'yaw']):
            rms_val = rms(diffs[i])
            percent = comp_percent(diffs[i], rms_val, par)
            unit = "m" if 'pos' in name else "deg"
            print(f"{name} {axis}: RMS={rms_val:.4f}({unit}), percent < {par}*RMS: {percent:.2f}%")

    for name, diffs, unit in [
        ("SITL pos norm", sitl_pos_norm_diffs, "m"),
        ("SITL att norm", sitl_att_norm_diffs, "deg"),
        ("HITL pos norm", hitl_pos_norm_diffs, "m"),
        ("HITL att norm", hitl_att_norm_diffs, "deg"),
    ]:
        rms_val = rms(diffs)
        percent = comp_percent(diffs, rms_val, par)
        print(f"{name}: RMS={rms_val:.4f}({unit}), percent < {par}*RMS: {percent:.2f}%")

if __name__ == "__main__":
    main()