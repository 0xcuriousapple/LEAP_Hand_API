import urx
import time
import math
import numpy as np
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import transforms3d as tf

# UR5e setup
robot_ip = "192.168.0.175"

# Leap setup
class LeapNode:
    def __init__(self):
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))

        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM13', 4000000)
                self.dxl_client.connect()
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def read_pos(self):
        return self.dxl_client.read_pos()

    def read_vel(self):
        return self.dxl_client.read_vel()

    def read_cur(self):
        return self.dxl_client.read_cur()

def clear_protective_stop(robot):
    """Clear protective stop and restart the robot."""
    print("Clearing protective stop...")
    robot.send_program("def clearProtectiveStop():\n\t"
                       "while (True):\n\t\t"
                       "if (read_input_boolean_register(24)):\n\t\t\t"
                       "textmsg('Protective Stop Cleared')\n\t\t\t"
                       "break\n\t\t"
                       "end\n\t"
                       "end\n"
                       "end\n")
    time.sleep(2)  # Give the robot some time to clear the stop
    robot.send_program("textmsg('Protective Stop Cleared')")

def move_robot(cartesian_position, euler_orientation):
    try:
        # Connect to the robot
        print(f"Connecting to robot at {robot_ip}...")
        robot = urx.Robot(robot_ip)
        print("Connected to robot.")

        # Increase the timeout for receiving data
        robot.secmon.socket_timeout = 5

        # Combine Cartesian position and Euler orientation
        pose = cartesian_position + euler_orientation
        print(f"Moving to new pose: {pose}")
        robot.movel(pose, acc=0.2, vel=0.1)
        print("Move command sent. Waiting for robot to reach position...")

        # Wait for the robot to reach the position
        time.sleep(5)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if 'robot' in locals():
            robot.close()
            print("Closed robot connection.")

def remap_positions(joint_positions):
    mapping = [6, 4, 5, 7, 10, 8, 9, 11, 14, 12, 13, 15, 0, 1, 2, 3]
    remapped_positions = [0] * 16
    for i, pos in enumerate(mapping):
        remapped_positions[pos] = joint_positions[i]
    return remapped_positions

def quaternion_to_euler_array(quaternion):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    Roll is rotation around the x-axis in radians (counterclockwise)
    Pitch is rotation around the y-axis in radians (counterclockwise)
    Yaw is rotation around the z-axis in radians (counterclockwise)
    
    Args:
        quaternion (list or tuple): Quaternion in the form [x, y, z, w]
        
    Returns:
        list: Euler angles in the form [roll, pitch, yaw]
    """
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]

def main():
    leap_hand = LeapNode()
    while True:
        try:
            user_input = input("Enter x, y, z, qx, qy, qz, qw, and 16 joint positions, separated by spaces (or type 'exit' to quit): ")
            if user_input.lower() == 'exit':
                break
            values = [float(i) for i in user_input.split()]
            if len(values) != 23:
                print("Please enter exactly 23 values.")
                continue
            cartesian_position = values[:3]
            quaternion = values[3:7]
            joint_positions = values[7:]

            # Convert quaternion to Euler angles (radians)
            euler_orientation = quaternion_to_euler_array(quaternion)

            # Remap joint positions
            remapped_positions = remap_positions(joint_positions)

            print(cartesian_position, euler_orientation)
            move_robot(cartesian_position, list(euler_orientation))
            leap_hand.set_allegro(remapped_positions)
            print("Position: " + str(leap_hand.read_pos()))
            time.sleep(0.03)
        except ValueError:
            print("Invalid input. Please enter numeric values.")

if __name__ == "__main__":
    main()
