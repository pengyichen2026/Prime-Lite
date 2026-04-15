#!/bin/bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 down 2>/dev/null
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 down 2>/dev/null
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 down 2>/dev/null
sudo ip link set can3 up type can bitrate 1000000
sudo ip link set can4 down 2>/dev/null
sudo ip link set can4 up type can bitrate 1000000
