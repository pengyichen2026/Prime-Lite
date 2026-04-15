"""
discover_actuators.py

检测 CAN 总线上存在的电机
"""

import argparse

from robstride_dynamics import RobstrideBus

parser = argparse.ArgumentParser()
parser.add_argument("--channel", "-c", type=str, default="", help="单独扫描某个 CAN 通道，例如 can2")
parser.add_argument("--start-can", type=int, default=1, help="扫描起始 CAN 编号（默认 1）")
parser.add_argument("--end-can", type=int, default=4, help="扫描结束 CAN 编号（默认 4）")
parser.add_argument("--start-id", type=int, default=1, help="电机 ID 起始值")
parser.add_argument("--end-id", type=int, default=50, help="电机 ID 结束值")
args = parser.parse_args()

def scan_channel(channel: str, start_id: int, end_id: int):
	bus = RobstrideBus(channel=channel, motors={})
	bus.connect()
	ids = bus.scan_channel(channel, start_id=start_id, end_id=end_id)
	bus.disconnect()
	return ids

if args.channel:
	ids = scan_channel(args.channel, args.start_id, args.end_id)
	print({args.channel: ids})
else:
	results = {}
	for can_index in range(args.start_can, args.end_can + 1):
		channel = f"can{can_index}"
		results[channel] = scan_channel(channel, args.start_id, args.end_id)
	print(results)

print("Program terminated.")
