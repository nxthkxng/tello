#!/bin/bash

echo "=================================================="
echo "🛫 กำลังเปิด Tello Driver และ Auto Height Control"
echo "=================================================="

source ~/tello_ws/install/setup.bash

gnome-terminal -- bash -c "source ~/tello_ws/install/setup.bash && ros2 launch tello_driver teleop_launch.py; exec bash"
echo "⏳ กำลังรอระบบ Tello Driver เตรียมพร้อม (4 วินาที)..."
sleep 4

python3 tello_auto_height.py