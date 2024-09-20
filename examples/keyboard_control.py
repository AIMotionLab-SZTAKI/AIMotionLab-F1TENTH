from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE


# connect to the vehicle
car_1 = F1Client(car_ID="JoeBush1") # (host="192.168.2.62", port=8069) is also possible

# set the mode to manual
car_1.set_mode(mode = CONTROLLER_MODE.MANUAL)

# control the vehicle using the keyboard
car_1.keyboard_control(d_max=.2, delta_max=.5)

# reset mode
car_1.set_mode(mode = CONTROLLER_MODE.IDLE)