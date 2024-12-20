# START F1TENTH
1. Plug in battery and wait for the system to boot
2. In terminal:
ssh f1tenth@192.168.2.62
pw: 123456
3. Run the following commands on the car system
`cd aimotion-f1tenth-system
source venv/bin/activate
source install/setup.bash
ros2 launch vehicle_control vehicle_launch.py`
