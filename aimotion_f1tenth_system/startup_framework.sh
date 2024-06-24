cd aimotion-f1tenth-python
export LD_LIBRARY_PATH=~/acados/lib/
export ACADOS_SOURCE_DIR=~/acados/
source install/setup.bash
ros2 launch vehicle_control vehicle_launch.py