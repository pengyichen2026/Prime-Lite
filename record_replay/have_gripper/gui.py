import tkinter as tk
from tkinter import messagebox
import subprocess
import threading
import os

current_path = os.path.abspath(__file__)
BASE_DIR = os.path.dirname(current_path)
ARM_DIR = os.path.join(BASE_DIR, "arm")

record_process = None  # 用于保存录制进程
action_counter = 1     # 动作计数器，从1开始

def run_command(cmd, cwd=None):
    try:
        subprocess.run(cmd, shell=True, cwd=cwd)
    except Exception as e:
        print(f"Error: {e}")

def run_in_thread(cmd, cwd=None):
    thread = threading.Thread(target=run_command, args=(cmd, cwd))
    thread.start()

# 初始化
def initialize():
    print("Running canup.sh...")
    run_command("bash canup.sh", cwd=BASE_DIR)
    print("Discovering actuators...")
    run_command("python3 discover_actuators.py -c=can0", cwd=BASE_DIR)
    run_command("python3 discover_actuators.py -c=can1", cwd=BASE_DIR)

# ======================
# 录制与回放逻辑
# ======================

def start_recording():
    global record_process, action_counter
    if record_process is None:
        # 使用当前计数器作为文件名的一部分
        action_name = f"record_{action_counter}"
        print(f"开始录制动作: {action_name}...")
        
        # Popen 启动异步进程
        record_process = subprocess.Popen(
            f"python3 record_motion.py -n {action_name}",
            shell=True,
            cwd=ARM_DIR
        )
        # 禁用开始按钮，防止重复点击
        btn_start.config(state=tk.DISABLED)
    else:
        print("录制已经在运行中！")

def stop_recording():
    global record_process, action_counter
    
    # 发送停止信号（根据你原有的逻辑：写文件标记）
    stop_flag = "/tmp/stop_record.flag"
    with open(stop_flag, "w") as f:
        f.write("stop")
    
    if record_process:
        record_process = None
        
    print(f"--- 停止录制成功！动作已保存为: 动作 {action_counter} ---")
    messagebox.showinfo("保存成功", f"动作已存储为：动作 {action_counter}")
    
    # 动作编号递增，并更新回放输入框的默认值
    action_counter += 1
    entry_replay_id.delete(0, tk.END)
    entry_replay_id.insert(0, str(action_counter - 1))
    
    # 恢复开始按钮
    btn_start.config(state=tk.NORMAL)

def replay():
    # 从输入框获取想要回放的动作编号
    target_id = entry_replay_id.get()
    if not target_id.isdigit():
        messagebox.showwarning("错误", "请输入有效的数字编号")
        return
    
    action_name = f"record_{target_id}"
    print(f"正在回放动作: {action_name}...")
    run_in_thread(f"python3 replay_motion.py -n {action_name}", cwd=ARM_DIR)

# ======================
# GUI 布局
# ======================

root = tk.Tk()
root.title("Robot Arm Control (v2)")
root.geometry("350x300")

# 录制控制区
frame_record = tk.LabelFrame(root, text="录制控制", padx=10, pady=10)
frame_record.pack(pady=10, fill="x", padx=20)

btn_start = tk.Button(frame_record, text="开始录制", command=start_recording, bg="#d4edda", height=1, width=15)
btn_start.pack(pady=5)

btn_stop = tk.Button(frame_record, text="结束录制", command=stop_recording, bg="#f8d7da", height=1, width=15)
btn_stop.pack(pady=5)

# 回放控制区
frame_replay = tk.LabelFrame(root, text="回放控制", padx=10, pady=10)
frame_replay.pack(pady=10, fill="x", padx=20)

# 编号选择
row_id = tk.Frame(frame_replay)
row_id.pack()
tk.Label(row_id, text="回放动作编号:").pack(side=tk.LEFT)
entry_replay_id = tk.Entry(row_id, width=5)
entry_replay_id.insert(0, "1")  # 默认显示1
entry_replay_id.pack(side=tk.LEFT, padx=5)

btn_replay = tk.Button(frame_replay, text="执行回放", command=replay, bg="#cce5ff", height=1, width=15)
btn_replay.pack(pady=10)

# 启动初始化线程
threading.Thread(target=initialize, daemon=True).start()

root.mainloop()
