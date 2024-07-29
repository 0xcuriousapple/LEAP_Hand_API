import urx
import time

# Define the robot's IP address
robot_ip = "192.168.0.175"

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

def move_robot():
    try:
        # Connect to the robot
        print(f"Connecting to robot at {robot_ip}...")
        robot = urx.Robot(robot_ip)
        print("Connected to robot.")

        # Increase the timeout for receiving data
        robot.secmon.socket_timeout = 5

        # Read the current Cartesian position
        print("Reading current Cartesian position...")
        current_cartesian_position = robot.getl()
        print("Current Cartesian Position: ", current_cartesian_position)

        # Define a small incremental movement in the z-direction
        new_cartesian_position = list(current_cartesian_position)
        new_cartesian_position[1] += 0.05 # Move up by 5 cm
        
        print(f"Moving to new Cartesian position: {new_cartesian_position}")

        # Move the robot to the new Cartesian position
        robot.movel(new_cartesian_position, acc=0.2, vel=0.1)
        time.sleep(5)
        new_cartesian_position[0] += 0.05
        robot.movel(new_cartesian_position, acc=0.2, vel=0.1)
        print("Move command sent. Waiting for robot to reach position...")

        # Wait for the robot to reach the position
        time.sleep(5)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if 'robot' in locals():
            robot.close()
            print("Closed robot connection.")

if __name__ == "__main__":
    move_robot()
