import urx

def get_current_position(robot_ip):
    # Connect to the UR robot
    robot = urx.Robot(robot_ip)

    try:
        # Get the current Cartesian position (pose) of the robot
        tcp_position = robot.getl()  # Returns the current position as [x, y, z, rx, ry, rz]

        # Print the current position
        print(f"Current TCP Position: {tcp_position}")

    finally:
        # Safely close the connection to the robot
        robot.close()

if __name__ == "__main__":
    # Replace '192.168.0.175' with your UR robot's IP address
    robot_ip = "192.168.0.175"
    get_current_position(robot_ip)
