import pandas as pd
import matplotlib.pyplot as plt
import time
import uart

# --- Constants ---
DEFAULT_UART_PORT = '/dev/ttyAMA0'  # Adjust if your RPi serial port is different
FIXED_VELOCITY = 0  # No movement, just to trigger logging
DURATION_SECONDS = 10  # Collect IMU data for 10 seconds

# --- Function to Send Command and Collect IMU Logs ---
def send_command_and_collect_logs(uart_comm, steering, gear, velocity, use_controller, duration_seconds):
    """
    Sends a command to the robot and collects IMU logs for the specified duration.
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
        use_controller=use_controller
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
                    # Parse data (note: values are scaled by 1000 for velocities/error/output, gyro is in micro-dps)
                    parsed_data = {
                        'timestamp_ms': int(parts[0]),
                        'desired_velocity': float(parts[1]) / 1000,  # Convert back to m/s
                        'actual_velocity': float(parts[2]) / 1000,
                        'error': float(parts[3]) / 1000,
                        'output': float(parts[4]) / 1000,
                        'gyro_z_udps': int(parts[5])  # Gyro Z in micro-dps
                    }
                    # Convert gyro to dps for plotting
                    parsed_data['gyro_z_dps'] = parsed_data['gyro_z_udps'] / 1000000.0
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

# --- Function to Plot IMU Data Over Time ---
def plot_imu_data(robot_responses_df):
    """
    Plots IMU gyro Z data over time.
    Args:
        robot_responses_df: Pandas DataFrame with robot log data.
    """
    if robot_responses_df.empty:
        print("No data to plot.")
        return
    
    print("Plotting IMU gyro Z over time...")
    plt.figure(figsize=(10, 6))
    plt.plot(robot_responses_df['timestamp_ms']/1000, robot_responses_df['gyro_z_dps'], label='Gyro Z (dps)', marker='.', color='blue')
    plt.xlabel('Czas (s)')
    plt.ylabel('Obrót w osi Z (stopnie na sekundę)')
    # plt.legend()
    # plt.title('IMU Gyro Z Readings Over 10 Seconds')
    plt.grid(True)
    plt.savefig("imu_plot.png")
    print("Plot saved as 'imu_plot.png'.")

# --- Main Function ---
def main():
    """
    Main function: Send a command to trigger IMU logging, collect for 10 seconds, then plot gyro data.
    """
    print("Starting IMU Test...")
    
    # Initialize UART
    uart_comm = uart.UARTCommunication(port=DEFAULT_UART_PORT)
    
    try:
        # Send a command to start logging (e.g., forward with no controller to keep motor running and logging)
        imu_df = send_command_and_collect_logs(
            uart_comm,
            steering='N',  # Right
            gear='F',      # Forward (or 'N' for neutral if you want no movement)
            velocity=FIXED_VELOCITY,  # 0 for no movement, or small value
            use_controller=0,  # No controller
            duration_seconds=7
        )
        
        # Send stop command
        print("Sending stop command...")
        uart_comm.send_command(
            steering='N',  # None
            gear='N',      # None
            velocity=0,
            use_controller=0
        )
        
        # Calculate average gyro Z
        if not imu_df.empty:
            average_gyro = imu_df['gyro_z_dps'].mean()
            print(f"Average gyro Z: {average_gyro:.3f} dps")
        
        # Plot IMU data
        plot_imu_data(imu_df)
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Closing UART communication.")
        uart_comm.close()
    
    print("IMU Test finished.")

if __name__ == "__main__":
    main()