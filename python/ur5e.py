import urx
import time

# Define the robot's IP address
robot_ip = "192.168.0.175"

# Define the minimum Z value to prevent end effector damage
min_z_value = -0.03744093181824759

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

        # Define the new starting Cartesian position
        start_cartesian_position = [-0.08891424614561083, -0.18015471959612878, 0.1349311753396791, -2.2987266762049336, -2.0880249687612285, 0.05292529829088585]
        print("Starting Cartesian Position:", start_cartesian_position)

        # Move the robot to the starting position
        print("Moving robot to the starting position...")
        robot.movel(start_cartesian_position, acc=0.2, vel=0.1)
        print("Robot is now at the starting position.")

        # Define movement increment (10 cm)
        movement_increment = 0.15  # 10 cm in meters

        # Define the five movement directions relative to the starting position (back removed)
        directions_five = {
            "up": [0, 0, movement_increment],
            "down": [0, 0, -movement_increment],
            "front": [-movement_increment, 0, 0],  # Move in the negative X direction
            "right": [0, movement_increment, 0],
            "left": [0, -movement_increment, 0]
        }

        # Perform the initial 5 directional moves
        for direction_name, direction_vector in directions_five.items():
            # Calculate the new position by adding the direction vector to the start position
            new_cartesian_position = [
                start_cartesian_position[0] + direction_vector[0],
                start_cartesian_position[1] + direction_vector[1],
                max(start_cartesian_position[2] + direction_vector[2], min_z_value),  # Safeguard for Z
                start_cartesian_position[3],  # rx remains the same
                start_cartesian_position[4],  # ry remains the same
                start_cartesian_position[5]   # rz remains the same
            ]

            if new_cartesian_position[2] < min_z_value:
                print(f"Skipping {direction_name} move because it would exceed Z limit.")
                continue

            print(f"Moving {direction_name} to position: {new_cartesian_position}")

            # Move the robot to the new position
            robot.movel(new_cartesian_position, acc=0.2, vel=0.1)
            print(f"Moved {direction_name}. Returning to starting position.")

            # Return to the starting position
            robot.movel(start_cartesian_position, acc=0.2, vel=0.1)
            print("Returned to starting position.")

            # Wait for a moment before the next move
            time.sleep(2)

        # Define the forward position (10 cm forward from starting position, in negative X direction)
        forward_position = [
            start_cartesian_position[0] - 0.2,
            start_cartesian_position[1],
            start_cartesian_position[2],
            start_cartesian_position[3],  # rx remains the same
            start_cartesian_position[4],  # ry remains the same
            start_cartesian_position[5]   # rz remains the same
        ]

        # Define the four movement directions relative to the forward position
        directions_four = {
            "up": [0, 0, movement_increment],
            "down": [0, 0, -movement_increment],
            "right": [0, movement_increment, 0],
            "left": [0, -movement_increment, 0]
        }

        # Perform the 4 directional moves from the forward position
        for direction_name, direction_vector in directions_four.items():
            # Calculate the new position by adding the direction vector to the forward position
            new_cartesian_position = [
                forward_position[0] + direction_vector[0],
                forward_position[1] + direction_vector[1],
                max(forward_position[2] + direction_vector[2], min_z_value),  # Safeguard for Z
                forward_position[3],  # rx remains the same
                forward_position[4],  # ry remains the same
                forward_position[5]   # rz remains the same
            ]

            if new_cartesian_position[2] < min_z_value:
                print(f"Skipping {direction_name} move from forward position because it would exceed Z limit.")
                continue

            print(f"Moving {direction_name} from forward position to: {new_cartesian_position}")

            # Move the robot to the new position
            robot.movel(new_cartesian_position, acc=0.2, vel=0.1)
            print(f"Moved {direction_name} from forward position. Returning to starting position.")

            # Return to the starting position
            robot.movel(start_cartesian_position, acc=0.2, vel=0.1)
            print("Returned to starting position.")

            # Wait for a moment before the next move
            time.sleep(2)

        # Define the forward position (30 cm forward from starting position, in negative X direction)
        forward_position_30cm = [
            start_cartesian_position[0] - 0.3,
            start_cartesian_position[1],
            start_cartesian_position[2],
            start_cartesian_position[3],  # rx remains the same
            start_cartesian_position[4],  # ry remains the same
            start_cartesian_position[5]   # rz remains the same
        ]

        # Define the two movement directions (left and right) relative to the 30 cm forward position
        directions_two = {
            "right": [0, movement_increment, 0],
            "left": [0, -movement_increment, 0]
        }

        # Perform the 2 directional moves from the 30 cm forward position
        for direction_name, direction_vector in directions_two.items():
            # Calculate the new position by adding the direction vector to the 30 cm forward position
            new_cartesian_position = [
                forward_position_30cm[0] + direction_vector[0],
                forward_position_30cm[1] + direction_vector[1],
                max(forward_position_30cm[2] + direction_vector[2], min_z_value),  # Safeguard for Z
                forward_position_30cm[3],  # rx remains the same
                forward_position_30cm[4],  # ry remains the same
                forward_position_30cm[5]   # rz remains the same
            ]

            if new_cartesian_position[2] < min_z_value:
                print(f"Skipping {direction_name} move from 30 cm forward position because it would exceed Z limit.")
                continue

            print(f"Moving {direction_name} from 30 cm forward position to: {new_cartesian_position}")

            # Move the robot to the new position
            robot.movel(new_cartesian_position, acc=0.2, vel=0.1)
            print(f"Moved {direction_name} from 30 cm forward position. Returning to starting position.")

            # Return to the starting position
            robot.movel(start_cartesian_position, acc=0.2, vel=0.1)
            print("Returned to starting position.")

            # Wait for a moment before the next move
            time.sleep(2)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if 'robot' in locals():
            robot.close()
            print("Closed robot connection.")

if __name__ == "__main__":
    move_robot()
