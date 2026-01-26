import PX4MavCtrlV4ROS as PX4MavCtrl
import time
import math

mav = PX4MavCtrl.PX4MavCtrler()
time.sleep(1)

height_z = -0.5

print('进入offboard并解锁')
mav.initOffboard()
time.sleep(5)

print('发送起飞命令')
mav.SendPosNED(0, 0, height_z, 0)
time.sleep(5)

print('PosE',mav.uavPosNED)
print('VelE',mav.uavVelNED)
print('Euler',mav.uavAngEular)
print('Quaternion',mav.uavAngQuatern)
print('Rate',mav.uavAngRate)

time.sleep(1)
print('Start control.')

# 下面开始你的控制算法
# mav.SendPosNED(x,y,z,yaw) 发送期望位置点，NED地球坐标系
# mav.SendVelNED(vx,vy,vz,yawrate) 发送速度，NED地球坐标系
# mav.SendVelFRD(vx,vy,vz,yawrate) 发送速度，FRD机体坐标系，通常而言，设定vx和yawrate即可，前进速度和偏航速度。
# mav.SendPosVelNED(PosE=[0,0,0],VelE=[0,0,0],yaw,yawrate) 同时控制飞机位置和速度

parameter_a = 2.0
parameter_b = 1.5
circle_times = 30
total_time = circle_times * 2
target_x = 0.0
target_y = 0.0

start_time = time.time()

while(time.time() - start_time < total_time):
    target_y = parameter_a * math.sin(
        (time.time() - start_time) / circle_times * 2 * math.pi )
    target_x = parameter_b * math.cos(
        (time.time() - start_time) / circle_times * 2 * math.pi ) * math.sin(
            (time.time() - start_time) / circle_times * 2 * math.pi )

    mav.SendPosNED(target_x, target_y, height_z, 0.0)
    time.sleep(0.1)

mav.SendPosNED(0.0, 0.0, height_z, 0.0)
time.sleep(5)

print('Landing')
mav.land()

