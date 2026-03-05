import math

def euler_yaw_to_quaternion(yaw):
    qx = 0.0                        # roll = 0 
    qy = 0.0                        # pitch = 0 
    qz = math.sin(yaw / 2.0)        # yaw
    qw = math.cos(yaw / 2.0)        # scalar
    return qx, qy, qz, qw