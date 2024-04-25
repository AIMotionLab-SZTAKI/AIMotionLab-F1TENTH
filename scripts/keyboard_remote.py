from aimotion_f1tenth_utils.F1Client import F1Client
from aimotion_f1tenth_utils.utils import CONTROLLER_MODE

car_1 = F1Client("192.168.2.62", 8069)


car_1.set_mode(mode = CONTROLLER_MODE.MANUAL)
car_1.keyboard_control()
