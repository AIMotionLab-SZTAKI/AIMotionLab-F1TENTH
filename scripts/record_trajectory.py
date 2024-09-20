import motioncapture
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("-i", "--ip", default="192.168.2.141")
parser.add_argument("-o", "--object_name", default="JoeBush1")
parser.add_argument("-d", "--devid", default=0)
args = vars(parser.parse_args())

mocap = motioncapture.MotionCaptureOptitrack(args["ip"])



def check_mocap(self):
    if not call_with_timeout(self.mocap.waitForNextFrame):
        raise Exception("Connection cannot be established with the MoCap server!")
    try:
        obj = self.mocap.rigidBodies[self.obj_name]
    except KeyError:
        raise Exception(f"The pose data of {self.obj_name} is not streamed by the MoCap server!")

def timestamp(self):
        return time.time() - self.start_time

def wait_frame_parse_pose(self):
    self.mocap.waitForNextFrame()
    try:
        obj = self.mocap.rigidBodies[self.obj_name]
        yaw = quat_2_yaw([obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w])
        return array.array("f", [self.timestamp()] + [yaw] + list(obj.position))
    except KeyError:  # not in frame
        return None
    
def call_with_timeout(func, timeout=5):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        future = executor.submit(func)
        try:
            future.result(timeout=timeout)
            return True
        except concurrent.futures.TimeoutError:
            return False
        
        
def quat_2_yaw(quat: List[float]):
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw