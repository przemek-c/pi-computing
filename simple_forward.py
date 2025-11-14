import pandas as pd
import matplotlib.pyplot as plt
import time
import uart

# --- Constants ---
DEFAULT_UART_PORT = '/dev/ttyAMA0'  # Adjust if your RPi serial port is different
FIXED_VELOCITY = 0.4  # Fixed velocity value (e.g., 0.4 m/s)
WAIT_DURATION_SECONDS = 10  # Wait 3 seconds before stopping

# --- Function to Send Command and Collect Logs ---
def send_command_and_collect_logs(uart_comm, steering, gear, velocity, use_controller, duration_seconds):
    """
    Sends a command to the robot and collects logs for the specified duration.
    Args:
        uart_comm: UARTCommunication instance.
        steering: Steering command ('S' for straight, etc.).
        gear: Gear command ('F' for forward, etc.).
        velocity: Velocity value.
        use_controller: 0 or 1 for controller usage.
        duration_seconds: How long to collect logs.
    Returns:
        Pandas DataFrame of collected responses.
    """
    print(f"Sending command: Steering={steering}, Gear={gear}, Velocity={velocity}, UseController={use_controller}")
    uart_comm.send_command(
        steering=steering,
        gear=gear,
        velocity=velocity,
        use_controller=use_controller,
        lifting='N'
    )
    
    robot_log_data = []
    start_time = time.monotonic()
    
    while (time.monotonic() - start_time) < duration_seconds:
        if uart_comm.serial_port.in_waiting > 0:
            response_line = uart_comm.serial_port.readline().decode('ascii', errors='ignore').strip()
            print(f"Robot response: {response_line}")
            parts = response_line.split(',')
            if len(parts) >= 6:  # Minimum expected parts
                try:
                    parsed_data = {
                        'timestamp_ms': int(parts[0]),
                        'desired_velocity': float(parts[1]),
                        'actual_velocity': float(parts[2]),
                        'error': float(parts[3]),
                        'output': float(parts[4]),
                        'gyro_z': float(parts[5])
                    }
                    robot_log_data.append(parsed_data)
                except (ValueError, IndexError) as e:
                    print(f"Error parsing response: {e}")
        time.sleep(0.01)  # Small delay to prevent busy-waiting
    
    # Adjust timestamps to start from 0
    if robot_log_data:
        initial_ts = robot_log_data[0]['timestamp_ms']
        for data in robot_log_data:
            data['timestamp_ms'] -= initial_ts
    
    return pd.DataFrame(robot_log_data)

# --- Function to Plot Velocity Over Time ---
def plot_velocity_over_time(robot_responses_df):
    """
    Plots desired and actual velocity over time.
    Args:
        robot_responses_df: Pandas DataFrame with robot log data.
    """
    if robot_responses_df.empty:
        print("No data to plot.")
        return
    
    print("Plotting velocity over time...")
    plt.figure(figsize=(10, 6))
    plt.plot(robot_responses_df['timestamp_ms'] / 1000, robot_responses_df['desired_velocity'] / 1000, label='Prędkość zadana', marker='.', linestyle='None')
    plt.plot(robot_responses_df['timestamp_ms'] / 1000, robot_responses_df['actual_velocity'] / 1000, label='Prędkość obliczona', marker='.', linestyle='None')
    plt.xlabel('Czas (s)')
    plt.ylabel('Prędkość (m/s)')
    plt.legend()
    plt.grid(True)
    plt.savefig("velocity_plot.png")
    print("Plot saved as 'velocity_plot.png'.")

# --- Main Function ---
def main():
    """
    Main function: Send forward command, wait 3 seconds collecting logs, send stop, then plot velocity.
    """
    print("Starting Simple Forward Test...")
    
    # Initialize UART
    uart_comm = uart.UARTCommunication(port=DEFAULT_UART_PORT)
    
    try:
        # Send forward command without controller
        forward_df = send_command_and_collect_logs(
            uart_comm,
            steering='S',  # Straight
            gear='F',      # Forward
            velocity=0.5*100,
            use_controller=1,  # With controller
            duration_seconds=20
        )
        
        # Send stop command
        print("Sending stop command...")
        uart_comm.send_command(
            steering='N',  # None
            gear='N',      # None
            velocity=0,
            use_controller=0,
            lifting='N'
        )
        
        # Combine data (if needed, but here we only have forward data)
        all_responses_df = forward_df
        
        # Calculate average speed (arithmetic mean)
        average_speed = (all_responses_df['actual_velocity'] / 1000).mean()
        print(f"Average speed (arithmetic mean): {average_speed:.3f} m/s")

        # Plot velocity
        plot_velocity_over_time(all_responses_df)
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Closing UART communication.")
        uart_comm.close()
    
    print("Test finished.")

if __name__ == "__main__":
    main()