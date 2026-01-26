import time
import math
import json
import socket
import select
import UE4CtrlAPI as UE4CtrlAPI

UE_REQ_TYPE = 1
UE_TARGET_ID = 1

LISTEN_IP = "0.0.0.0"
# LISTEN_IP = "192.168.3.21"
LISTEN_PORT = 16520

ue = UE4CtrlAPI.UE4CtrlAPI()
ue.reqCamCoptObj(UE_REQ_TYPE, [UE_TARGET_ID])
time.sleep(1.0)
ue.initUE4MsgRec()
time.sleep(1.0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))
sock.setblocking(False)

print(f"[UDP] listening on {LISTEN_IP}:{LISTEN_PORT} ...")

running = True
started = False

while running:
    r, _, _ = select.select([sock], [], [], 0.2)
    if not r:
        continue

    data, _addr = sock.recvfrom(8192)
    msg = json.loads(data.decode("utf-8"))

    mtype = msg.get("type", "")
    if mtype == "start":
        started = True
        print("[UDP] start received")
        continue

    if mtype == "end":
        print("[UDP] end received")
        running = False
        continue

    if mtype != "step":
        continue
    
    print(msg)

    target = msg.get("pos", [0.0, 0.0, 0.0])
    posE = target[:3]
    angEuler = msg.get("att_deg", [0.0, 0.0, 0.0])
    yaw_conv = math.radians(angEuler[2])
    yaw_origin = -(yaw_conv - math.pi / 2)
    angEuler[2] = math.degrees(yaw_origin)

    ue.sendUE4PosNew(
        copterID=UE_TARGET_ID,
        vehicleType=3,
        PosE=posE,
        AngEuler=angEuler,
        windowID=-1
    )