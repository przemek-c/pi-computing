import pandas as pd
import matplotlib.pyplot as plt
import time
import sys
import reeds_shepp as rs
import uart
import math  # Added for trigonometric functions and pi
'''
# Attempt to import project-specific modules
try:
    # Assuming reeds_shepp.py is in Model2/nav_with_no_graph/
    from nav_with_no_graph import reeds_shepp as rs
    # Assuming uart.py is in Model2/ and contains UARTCommunication class
    import uart
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Please ensure 'reeds_shepp.py' is in 'Model2/nav_with_no_graph/' relative to this script's parent directory,")
    print("and 'uart.py' is in 'Model2/' (the same directory as this script or its parent).")
    print("You might need to adjust PYTHONPATH or ensure __init__.py files are present if these are packages.")
    sys.exit(1)
'''
# --- Constants ---
FORWARD_VELOCITY = 2  # units/second
BACKWARD_VELOCITY = 1 # units/second
DEFAULT_UART_PORT = '/dev/ttyAMA0'  # Adjust if your RPi serial port is different
ROBOT_START_POINT = (0, 0, 0)  # Assuming robot starts at origin 
TURNING_RADIUS = 3  # Example turning radius in meters, adjust as needed

#1.30m in 4 seconds2

# --- 1. Terminal Communication ---
def get_terminal_input():
    """
    Prompts the user to enter coordinates (x, y, angle, use_controller) or 'q' to quit.
    Returns a tuple (x, y, angle, use_controller) or None if the user quits.
    """
    while True:
        try:
            user_input = input("Enter target coordinates (x, y, angle, use_controller (0 or 1)) separated by commas, or 'q' to quit: ")
            if user_input.lower() == 'q':
                return None
            parts = user_input.split(',')
            if len(parts) == 4:
                x = float(parts[0].strip())
                y = float(parts[1].strip())
                angle = float(parts[2].strip())
                use_controller = int(parts[3].strip())
                if use_controller not in [0, 1]:
                    print("Invalid value for use_controller. Must be 0 or 1.")
                    continue
                print(f"Target received: x={x}, y={y}, angle={angle}, use_controller={use_controller}")
                return x, y, angle, use_controller
            else:
                print("Invalid format. Expected: x, y, angle, use_controller (e.g., 1.5, 2.0, 90, 1)")
        except ValueError:
            print("Invalid input. Please ensure you enter numbers for coordinates, angle, and 0 or 1 for use_controller.")
        except Exception as e:
            print(f"An error occurred during input: {e}")

# --- 2. Path Generation ---
def generate_path_segment(start_point, end_point):
    """
    Generates a path segment using reeds_shepp.
    Args:
        start_point: Tuple (x, y, theta_degrees) for the start.
        end_point: Tuple (x, y, theta_degrees) for the target.
    Returns:
        A list of path objects from reeds_shepp, or None if path generation fails.
    """
    print(f"Generating path from {start_point} to {end_point}...")
    try:
        # The Plan.md implies ROUTE is just two points for this segment
        # rs.get_optimal_path is expected to return a list of path objects (segments)
        path_objects = rs.get_optimal_path(start_point, end_point)
        if not path_objects:
            print("Path generation returned no path.")
            return None
        # path_length = rs.path_length(path_objects) # If you need the total length
        # print(f"Generated path with total length: {path_length}")
        return path_objects
    except Exception as e:
        print(f"Error during path generation: {e}")
        return None

def create_path_dataframe(full_path, forward_vel, backward_vel):
    """
    Converts the list of path objects from Reeds-Shepp into a Pandas DataFrame
    and adds velocity and duration columns.
    Args:
        full_path: List of path objects (from generate_path_segment).
        forward_vel: Forward velocity.
        backward_vel: Backward velocity.
    Returns:
        A Pandas DataFrame with path details, or None if input is invalid.
    """
    if not full_path:
        print("No path objects to create DataFrame.")
        return None

    print("Creating DataFrame from path objects...")
    try:
        # Each 'p' in full_path is expected to be an object with
        # attributes like p.steering, p.gear, p.param (distance)
        df = pd.DataFrame([
            {'steering': p.steering, 'gear': p.gear, 'distance': p.param}
            for p in full_path
        ])

        if df.empty:
            print("DataFrame is empty after processing path objects.")
            return None

        # Add velocity column based on gear
        # Assumes rs.Gear.FORWARD and rs.Gear.BACKWARD are defined in reeds_shepp module
        df['velocity'] = df['gear'].apply(
            lambda g: forward_vel if g == rs.Gear.FORWARD else (backward_vel if g == rs.Gear.BACKWARD else 0)
        )

        # Calculate and add duration column
        # Avoid division by zero if velocity is 0 (e.g., for neutral gear or if path segment has 0 velocity)
        # Modify distance by turning radius for duration calculation
        df['duration_s'] = df.apply(
            lambda row: (row['distance'] * TURNING_RADIUS) / row['velocity'] if row['velocity'] != 0 else 0,
            axis=1
        )
        df['duration_ms'] = df['duration_s'] * 1000

        print("DataFrame created successfully:")
        print(df.head())
        return df
    except AttributeError as e:
        print(f"Error accessing attributes from path objects (e.g., p.steering, p.gear, p.param): {e}")
        print("Ensure the objects returned by reeds_shepp.get_optimal_path have these attributes.")
        return None
    except Exception as e:
        print(f"An error occurred during DataFrame creation: {e}")
        return None

# --- 3. Robot Communication ---
def communicate_with_robot(path_df, uart_port, use_controller_flag):
    """
    Sends commands from the path DataFrame to the robot via UART and logs responses.
    Args:
        path_df: Pandas DataFrame with path commands.
        uart_port: The serial port for UART communication.
        use_controller_flag: Integer (0 or 1) to indicate if MCU should use controller.
    Returns:
        A Pandas DataFrame of robot responses, or an empty DataFrame if no data.
    """
    if path_df is None or path_df.empty:
        print("Path DataFrame is empty. No commands to send to the robot.")
        return pd.DataFrame()

    print(f"Initializing UART communication on port {uart_port}...")
    # uart.UARTCommunication is expected from uart.py
    uart_comm = uart.UARTCommunication(port=uart_port)
    robot_log_data = []

    try:
        for index, row in path_df.iterrows():
            # Map DataFrame values to UART command parameters
            # Assumes rs.Steering enums are defined in reeds_shepp
            if row['steering'] == rs.Steering.LEFT:
                steering_char = 'L'
            elif row['steering'] == rs.Steering.RIGHT:
                steering_char = 'R'
            else:  # rs.Steering.STRAIGHT or other
                steering_char = 'S'
            
            # Assumes rs.Gear enums are defined in reeds_shepp
            gear_char = 'F' if row['gear'] == rs.Gear.FORWARD else ('B' if row['gear'] == rs.Gear.BACKWARD else 'N')
            
            velocity_int = int(row['velocity'])
            duration_ms_int = int(row['duration_ms'])

            print(f"Sending: Steering={steering_char}, Gear={gear_char}, Velocity={velocity_int}, Duration={duration_ms_int}ms, UseController={use_controller_flag}")
            # NOTE: uart.UARTCommunication.send_command will need to be updated
            # to accept the use_controller_flag parameter.
            uart_comm.send_command(
                steering=steering_char,
                gear=gear_char,
                velocity=velocity_int,
                use_controller=use_controller_flag
                # duration=duration_ms_int
            )
            
            start_read_time = time.monotonic()
            # do we calculate duration of all path segments?
            # or just the first one?
            
            command_duration_seconds = duration_ms_int / 1000.0
            
            while (time.monotonic() - start_read_time) < command_duration_seconds:
                if uart_comm.serial_port.in_waiting > 0:
                    response_line = uart_comm.serial_port.readline().decode('ascii', errors='ignore').strip()
                    print(f"Robot response: {response_line}")
                    parts = response_line.split(',')
                    # Based on C printf: %lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f,%ld
                    # HAL_GetTick(), desiredV, actualV, error, output, gyro_z, (optional extra_val)
                    if len(parts) >= 6: # Minimum expected parts
                        try:
                            parsed_data = {
                                'timestamp_ms': int(parts[0]),
                                'desired_velocity': float(parts[1]),
                                'actual_velocity': float(parts[2]),
                                'error': float(parts[3]),
                                'output': float(parts[4]),
                                'gyro_z': float(parts[5])
                            }
                            # if len(parts) == 7: # As per Plan.md C code, there's a 7th %ld
                            #     parsed_data['extra_val'] = int(parts[6])
                            robot_log_data.append(parsed_data)
                        except ValueError as ve:
                            print(f"Could not parse robot response parts: {parts}, Error: {ve}")
                        except IndexError as ie:
                            print(f"Index error parsing robot response parts: {parts}, Error: {ie}")

                time.sleep(0.01) # Small delay to prevent busy-waiting

        print("All commands sent. Sending STOP command.")
        # For STOP command, typically controller might be disengaged, sending 0.
        # Or, it could be the last state. Let's send 0 for simplicity.
        uart_comm.send_command(steering='N', gear='N', velocity=0, use_controller=0)

    except Exception as e:
        print(f"An error occurred during robot communication: {e}")
    finally:
        print("Closing UART communication.")
        uart_comm.close()

    return pd.DataFrame(robot_log_data)

def calculate_x_y_coords(robot_responses_df):
    # Calculate X, Y, Theta based on actual_velocity and gyro_z
    # Make a copy to safely add columns
    df_for_odometry = robot_responses_df.copy()
    
    # Initial state from ROBOT_START_POINT
    _x = ROBOT_START_POINT[0]
    _y = ROBOT_START_POINT[1]
    _theta_rad = math.radians(ROBOT_START_POINT[2]) # Convert initial angle to radians
    
    calculated_x_coords = []
    calculated_y_coords = []
    
    last_timestamp_ms = 0 # Assume system starts at t=0 with ROBOT_START_POINT state

    # Sort by timestamp just in case, though it should be ordered
    df_for_odometry = df_for_odometry.sort_values(by='timestamp_ms').reset_index(drop=True)

    for i in range(len(df_for_odometry)):
        row = df_for_odometry.iloc[i]
        current_timestamp_ms = row['timestamp_ms']
        
        # dt is time elapsed since last measurement, or from t=0 for the first measurement
        dt_s = (current_timestamp_ms - last_timestamp_ms) / 1000.0
        
        actual_velocity = row['actual_velocity']
        gyro_z_dps = row['gyro_z'] # Assuming gyro_z is in degrees per second
        gyro_z_rad_s = math.radians(gyro_z_dps)

        # Update position using velocity (from current row) and theta (from previous step's end state)
        # This is a common Euler integration approach: x_k+1 = x_k + v_k * cos(theta_k) * dt
        _x += actual_velocity * math.cos(_theta_rad) * dt_s
        _y += actual_velocity * math.sin(_theta_rad) * dt_s
        
        # Update theta using gyro from current row over dt
        # theta_k+1 = theta_k + omega_k * dt
        _theta_rad += gyro_z_rad_s * dt_s
        # Normalize theta to be within [-pi, pi]
        _theta_rad = (_theta_rad + math.pi) % (2 * math.pi) - math.pi

        calculated_x_coords.append(_x)
        calculated_y_coords.append(_y)
        
        last_timestamp_ms = current_timestamp_ms
            
    # Add calculated positions to the original DataFrame for plotting
    robot_responses_df['x_pos'] = calculated_x_coords
    robot_responses_df['y_pos'] = calculated_y_coords

# --- 4. Plotting ---
def plot_robot_data(robot_responses_df):
    """
    Plots data collected from the robot, including estimated X, Y position.
    Args:
        robot_responses_df: Pandas DataFrame with robot log data.
    """
    if robot_responses_df is None or robot_responses_df.empty:
        print("No robot data collected or DataFrame is empty. Skipping plotting.")
        return

    print("Plotting robot data...")

    calculate_x_y_coords(robot_responses_df)

    try:
        plt.figure(figsize=(15, 18)) # Adjusted figure size for 5 plots

        # Plot 1: Desired vs Actual Velocity
        plt.subplot(5, 1, 1) # Changed to 5 rows
        plt.plot(robot_responses_df['timestamp_ms'], robot_responses_df['desired_velocity'], label='Desired Velocity', marker='.')
        plt.plot(robot_responses_df['timestamp_ms'], robot_responses_df['actual_velocity'], label='Actual Velocity', marker='.')
        plt.xlabel('Time (ms)')
        plt.ylabel('Velocity')
        plt.legend()
        plt.title('Velocity Over Time')
        plt.grid(True)

        # Plot 2: Control Error
        plt.subplot(5, 1, 2) # Changed to 5 rows
        plt.plot(robot_responses_df['timestamp_ms'], robot_responses_df['error'], label='Error', color='red', marker='.')
        plt.xlabel('Time (ms)')
        plt.ylabel('Error')
        plt.legend()
        plt.title('Control Error Over Time')
        plt.grid(True)

        # Plot 3: Gyroscope Z
        plt.subplot(5, 1, 3) # Changed to 5 rows
        plt.plot(robot_responses_df['timestamp_ms'], robot_responses_df['gyro_z'], label='Gyro Z (dps)', color='green', marker='.')
        plt.xlabel('Time (ms)')
        plt.ylabel('Gyro Z (dps)')
        plt.legend()
        plt.title('Gyroscope Z Over Time')
        plt.grid(True)
        
        # Plot 4: Controller Output (if available and desired)
        if 'output' in robot_responses_df.columns:
            plt.subplot(5, 1, 4) # Changed to 5 rows
            plt.plot(robot_responses_df['timestamp_ms'], robot_responses_df['output'], label='Controller Output', color='purple', marker='.')
            plt.xlabel('Time (ms)')
            plt.ylabel('Controller Output')
            plt.legend()
            plt.title('Controller Output Over Time')
            plt.grid(True)
        
        # Plot 5: Estimated X and Y Position Over Time
        if 'x_pos' in robot_responses_df.columns and 'y_pos' in robot_responses_df.columns:
            plt.subplot(5, 1, 5) # New subplot for X, Y position
            plt.plot(robot_responses_df['timestamp_ms'], robot_responses_df['x_pos'], label='Estimated X Position (units)', marker='.')
            plt.plot(robot_responses_df['timestamp_ms'], robot_responses_df['y_pos'], label='Estimated Y Position (units)', marker='.')
            plt.xlabel('Time (ms)')
            plt.ylabel('Position (units)')
            plt.legend()
            plt.title('Estimated Robot Position Over Time')
            plt.grid(True)

        plt.tight_layout()
        plt.savefig("robot_data_plot.png") # Save the figure
        # plt.show()
        print("Plot saved.")
    except KeyError as e:
        print(f"Plotting error: A required column is missing from the robot_responses_df: {e}")
        print(f"Available columns: {robot_responses_df.columns.tolist()}")
    except Exception as e:
        print(f"An error occurred during plotting: {e}")

# --- Main Application Logic ---
def main():
    """
    Main function to run the robot control sequence.
    """
    print("Starting Robot Control Application...")

    # 1. Get terminal input for target coordinates
    target_input_tuple = get_terminal_input()
    if target_input_tuple is None:
        print("User quit. Exiting application.")
        return

    target_x, target_y, target_angle, use_controller_flag = target_input_tuple

    # 2. Generate path segment
    # Assuming the robot always starts from a fixed point for each new command sequence
    path_objects = generate_path_segment(ROBOT_START_POINT, (target_x, target_y, target_angle))
    if not path_objects:
        print("Failed to generate path. Exiting.")
        return

    # 3. Create DataFrame from path
    path_df = create_path_dataframe(path_objects, FORWARD_VELOCITY, BACKWARD_VELOCITY)
    if path_df is None or path_df.empty:
        print("Failed to create path DataFrame. Exiting.")
        return

    # 4. Communicate with robot
    robot_responses_df = communicate_with_robot(path_df, DEFAULT_UART_PORT, use_controller_flag)

    # 5. Plot data from robot
    if not robot_responses_df.empty:
        plot_robot_data(robot_responses_df)
    else:
        print("No data received from the robot to plot.")

    print("Robot Control Application finished.")

if __name__ == "__main__":
    main()
