import pickle
import socket
import time
import numpy as np
from oculus_reader.reader import OculusReader
from scipy.spatial.transform import Rotation as R

# 1. 初始化
oculus = OculusReader()
DEST_IP = "127.0.0.1"  
DEST_PORT = 11005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def matrix_to_bhl_pose(matrix):
    """
    将 Oculus 变换矩阵转换为 BHL 期望的坐标系和格式
    Oculus: X-右, Y-上, Z-后
    BHL 期望 (典型 MuJoCo): X-前, Y-左, Z-上
    """
    # --- 1. 位置映射 (Position Mapping) ---
    # 原始 Oculus 位置
    raw_pos = matrix[:3, 3]
    
    # 坐标轴转换逻辑：
    # BHL_X = -Oculus_Z (手柄向前推，Z减小，对应机器人X增加)
    # BHL_Y = -Oculus_X (手柄向左移，X减小，对应机器人Y增加)
    # BHL_Z =  Oculus_Y (手柄向上提，Y增加，对应机器人Z增加)
    pos = [-raw_pos[2], -raw_pos[0], raw_pos[1]]

    # --- 2. 旋转映射 (Rotation Mapping) ---
    rot_matrix = matrix[:3, :3]
    r = R.from_matrix(rot_matrix)
    
    # 同样需要对旋转矩阵进行基底变换，这里先转为欧拉角进行坐标系补偿
    # 或者直接转为四元数。为了稳定，我们先提取标准四元数
    quat_xyzw = r.as_quat()
    
    # 重新排列为 WXYZ
    # 注意：如果动作依然方向不对，可能需要在这里乘以一个校准矩阵
    quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
    
    return pos, quat_wxyz

print(f"🚀 转发中... Quest -> BHL (UDP {DEST_PORT})")
print("重要提示:")
print("1. 保持手柄静止后启动 BHL 控制脚本")
print("2. 启动后按下手柄侧键/A键进行 IK 重置(Rebase)")
print("3. 当前转发频率: 30Hz (已降低以适配 CAN 总线带宽)")

try:
    last_print_time = time.time()
    while True:
        transforms, buttons = oculus.get_transformations_and_buttons()
        
        if not transforms:
            time.sleep(0.01)
            continue

        states = {}
        
        # 处理左手
        if 'l' in transforms:
            pos_l, quat_l = matrix_to_bhl_pose(transforms['l'])
            states["left"] = {
                "relative_location": pos_l,
                "relative_orientation": quat_l
            }
            
        # 处理右手
        if 'r' in transforms:
            pos_r, quat_r = matrix_to_bhl_pose(transforms['r'])
            states["right"] = {
                "relative_location": pos_r,
                "relative_orientation": quat_r
            }

        if states:
            data_packet = pickle.dumps(states)
            sock.sendto(data_packet, (DEST_IP, DEST_PORT))
            
            # 降低打印频率，减少终端 I/O 对性能的影响
            current_time = time.time()
            if current_time - last_print_time > 0.5:
                if "left" in states:
                    lp = states["left"]["relative_location"]
                    print(f"发送中... 左手(BHL坐标系): X:{lp[0]:.2f}, Y:{lp[1]:.2f}, Z:{lp[2]:.2f}", end='\r')
                last_print_time = current_time

        # 关键修改：改为 0.033 (约 30Hz)
        # 这样不会让 CAN 总线因为处理过载而爆出 Invalid Device ID
        time.sleep(0.033) 

except KeyboardInterrupt:
    print("\n已安全停止转发。")
