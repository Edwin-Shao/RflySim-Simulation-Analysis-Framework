# import time
# import PX4MavCtrlV4ROS as PX4MavCtrl
# mav = PX4MavCtrl.PX4MavCtrler()
# time.sleep(1)
# mav.initOffboard()
# time.sleep(3)
# print("发送起飞命令")
# mav.SendPosNED(0, 0, -2.5, 0)
# time.sleep(5)
# mav.SendPosNED(5,1,-2.5,0)
# time.sleep(5)
# print(mav.uavPosNED)
# mav.land()

# send_pos = [5, 1, 2.5]
# rec_pos = [-0.98, 5.04, -2.44]
# real_pos = rec_pos

import json
import matplotlib.pyplot as plt

# 1. 读取json文件
json_path = r'd:\code\NIMTE\rflysim\flylog\FC_20260122_152558.json'
with open(json_path, 'r', encoding='utf-8') as f:
    data = json.load(f)

# 2. 提取三维坐标
xs, ys, zs = [], [], []
for row in data:
    pos = row.get('pos')
    if pos and len(pos) >= 3:
        xs.append(pos[0])
        ys.append(pos[1])
        zs.append(pos[2])

# 3. 画三维轨迹
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xs, ys, zs, 'g-', label='real path')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Path from FC_20260122_152558.json')
ax.legend()
plt.show()