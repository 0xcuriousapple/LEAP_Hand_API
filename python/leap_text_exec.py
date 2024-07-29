#!/usr/bin/env python3
import json
import numpy as np
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time

class LeapNode:
    def __init__(self):
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))

        self.motors = motors = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
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
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kD * 0.75), 80, 2)
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
    
def remap_positions(joint_positions):
    mapping = [6, 4, 5, 7, 10, 8, 9, 11, 14, 12, 13, 15, 0, 1, 2, 3]
    remapped_positions = [0] * 16
    for i, pos in enumerate(mapping):
        remapped_positions[pos] = joint_positions[i]
    return remapped_positions

def main(file_path):
    leap_hand = LeapNode()

    # Read the JSON file
    with open(file_path, 'r') as file:
        data = json.load(file)
    
    # Process the data
    result_array = []
    for key in sorted(data.keys(), key=int):
        try:
            # Convert elements to float
            elements = [float(i) for i in data[key]]
            
            if len(elements) == 16:
                # Append the 16 elements to the result array
                result_array.append(elements)
            else:
                print(f"Entry {key} skipped, doesn't have exactly 16 elements.")
                
        except ValueError as e:
            print(f"Non-numeric value found in entry {key}, skipping: {e}")

    # Iterate over each position and set it with a time buffer
    for position in result_array:
        leap_hand.set_allegro(remap_positions(position))
        print("Position: " + str(leap_hand.read_pos()))
        time.sleep(1)  # Time buffer of 1 second between each position

if __name__ == "__main__":
    file_path = 'test.json'
    main(file_path)
