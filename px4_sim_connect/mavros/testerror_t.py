import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def load_json(path):
    with open(path, 'r') as f:
        data = json.load(f)
    # 兼容单条或多条数据
    if isinstance(data, dict):
        data = [data]
    return data

def extract(data):
    def get_pos(d):
        p = d['pos']
        if isinstance(p, list) and len(p) > 0 and isinstance(p[0], list):
            return p[0]
        return p
    pos = np.array([get_pos(d) for d in data])
    target = np.array([d['target'] for d in data])
    att = np.array([d['att_deg'] for d in data])
    t = np.array([d['t'] for d in data])
    return t, pos, target, att

def calc_errors(pos, target, att):
    pos_err = pos - target[:, :3]  # 只取target的前三个分量
    pos_err_norm = np.linalg.norm(pos_err, axis=1)
    att_err = att  # 与0比较
    att_err_norm = np.linalg.norm(att, axis=1)
    return pos_err, pos_err_norm, att_err, att_err_norm

def plot_all(ts, poss, targets, pos_errs, pos_err_norms, att_errs, att_err_norms, labels):
    # 轨迹
    fig = plt.figure(figsize=(15,10))
    ax = fig.add_subplot(221, projection='3d')
    for pos, target, label in zip(poss, targets, labels):
        ax.plot(pos[:,0], pos[:,1], pos[:,2], label=f'{label} pos')
        ax.plot(target[:,0], target[:,1], target[:,2], '--', label=f'{label} target')
    ax.set_title('3D Trajectory')
    ax.legend()
    # 使三个轴刻度一致
    all_targets3 = [t[:, :3] for t in targets]
    all_pos = np.concatenate(poss + all_targets3, axis=0)
    xyz_min = all_pos.min(axis=0)
    xyz_max = all_pos.max(axis=0)
    xyz_range = xyz_max - xyz_min
    max_range = xyz_range.max()
    mid = (xyz_max + xyz_min) / 2
    for i, axis in enumerate(['x', 'y', 'z']):
        getattr(ax, f'set_{axis}lim')(mid[i] - max_range/2, mid[i] + max_range/2)
    try:
        ax.set_box_aspect([1,1,1])
    except Exception:
        pass  # 低版本matplotlib不支持

    # 位置误差
    plt.subplot(222)
    for t, pos_err_norm, label in zip(ts, pos_err_norms, labels):
        plt.plot(t, pos_err_norm, label=label)
        # 标注最大和均值
        max_idx = np.argmax(pos_err_norm)
        plt.annotate(f"max={pos_err_norm[max_idx]:.3f}", (t[max_idx], pos_err_norm[max_idx]),
                     textcoords="offset points", xytext=(0,10), ha='center', fontsize=8, color='red')
        mean_val = np.mean(pos_err_norm)
        plt.annotate(f"mean={mean_val:.3f}", (t[len(t)//2], mean_val),
                     textcoords="offset points", xytext=(0,10), ha='center', fontsize=8, color='blue')
    plt.title('Position Error (Euclidean)')
    plt.xlabel('t')
    plt.ylabel('Error (m)')
    plt.legend()

    # 位置分量误差
    plt.subplot(223)
    for t, pos_err, label in zip(ts, pos_errs, labels):
        for i, comp in enumerate('xyz'):
            plt.plot(t, pos_err[:,i], label=f'{label} {comp}')
            # 标注最大
            max_idx = np.argmax(np.abs(pos_err[:,i]))
            plt.annotate(f"max={pos_err[max_idx,i]:.3f}", (t[max_idx], pos_err[max_idx,i]),
                         textcoords="offset points", xytext=(0,8), ha='center', fontsize=7)
    plt.title('Position Component Error')
    plt.xlabel('t')
    plt.ylabel('Error (m)')
    plt.legend(fontsize=8)

    # 姿态误差
    plt.subplot(224)
    for t, att_err_norm, label in zip(ts, att_err_norms, labels):
        plt.plot(t, att_err_norm, label=label)
        max_idx = np.argmax(att_err_norm)
        plt.annotate(f"max={att_err_norm[max_idx]:.2f}", (t[max_idx], att_err_norm[max_idx]),
                     textcoords="offset points", xytext=(0,10), ha='center', fontsize=8, color='red')
        mean_val = np.mean(att_err_norm)
        plt.annotate(f"mean={mean_val:.2f}", (t[len(t)//2], mean_val),
                     textcoords="offset points", xytext=(0,10), ha='center', fontsize=8, color='blue')
    plt.title('Attitude Error (deg)')
    plt.xlabel('t')
    plt.ylabel('Error (deg)')
    plt.legend()

    plt.tight_layout()
    plt.show()

    # 位置误差
    plt.subplot(222)
    for t, pos_err_norm, label in zip(ts, pos_err_norms, labels):
        plt.plot(t, pos_err_norm, label=label)
    plt.title('Position Error (Euclidean)')
    plt.xlabel('t')
    plt.ylabel('Error (m)')
    plt.legend()

    # 位置分量误差
    plt.subplot(223)
    for t, pos_err, label in zip(ts, pos_errs, labels):
        plt.plot(t, pos_err[:,0], label=f'{label} x')
        plt.plot(t, pos_err[:,1], label=f'{label} y')
        plt.plot(t, pos_err[:,2], label=f'{label} z')
    plt.title('Position Component Error')
    plt.xlabel('t')
    plt.ylabel('Error (m)')
    plt.legend()

    # 姿态误差
    plt.subplot(224)
    for t, att_err_norm, label in zip(ts, att_err_norms, labels):
        plt.plot(t, att_err_norm, label=label)
    plt.title('Attitude Error (deg)')
    plt.xlabel('t')
    plt.ylabel('Error (deg)')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # 替换为你的三个文件路径
    files = [
        'D:\\code\\NIMTE\\rflysim\\flylog\\FC_20260126_hitl1.json',
        'D:\\code\\NIMTE\\rflysim\\flylog\\FC_20260126_lidar1.json',
        'D:\\code\\NIMTE\\rflysim\\flylog\\FC_20260126_sitl1.json'
    ]
    labels = ['hitl1', 'lidar1', 'sitl1']
    ts, poss, targets, att_s = [], [], [], []
    pos_errs, pos_err_norms, att_errs, att_err_norms = [], [], [], []
    for f in files:
        data = load_json(f)
        t, pos, target, att = extract(data)
        pos_err, pos_err_norm, att_err, att_err_norm = calc_errors(pos, target, att)
        ts.append(t)
        poss.append(pos)
        targets.append(target)
        att_s.append(att)
        pos_errs.append(pos_err)
        pos_err_norms.append(pos_err_norm)
        att_errs.append(att_err)
        att_err_norms.append(att_err_norm)
    plot_all(ts, poss, targets, pos_errs, pos_err_norms, att_errs, att_err_norms, labels)