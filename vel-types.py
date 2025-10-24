# ...existing code...

# --- Constants ---
# New velocities for different move types and distances
# Short distance: < 0.3m, Long distance: >= 0.3m
FORWARD_STRAIGHT_SHORT_VELOCITY = 0.17  # Reduced for short straight forward (e.g., 80% of long)
FORWARD_STRAIGHT_LONG_VELOCITY = 0.215  # Full speed for long straight forward
FORWARD_LEFT_SHORT_VELOCITY = 0.14  # Reduced for short left turns
FORWARD_LEFT_LONG_VELOCITY = 0.17  # Reduced for long left turns
FORWARD_RIGHT_SHORT_VELOCITY = 0.14  # Reduced for short right turns
FORWARD_RIGHT_LONG_VELOCITY = 0.17  # Reduced for long right turns
BACKWARD_STRAIGHT_SHORT_VELOCITY = 0.20  # Reduced for short straight backward
BACKWARD_STRAIGHT_LONG_VELOCITY = 0.25  # Full speed for long straight backward
BACKWARD_LEFT_SHORT_VELOCITY = 0.16  # Reduced for short left turns backward
BACKWARD_LEFT_LONG_VELOCITY = 0.20  # Reduced for long left turns backward
BACKWARD_RIGHT_SHORT_VELOCITY = 0.16  # Reduced for short right turns backward
BACKWARD_RIGHT_LONG_VELOCITY = 0.20  # Reduced for long right turns backward
DEFAULT_UART_PORT = '/dev/ttyAMA0'  # Adjust if your RPi serial port is different
ROBOT_START_POINT = (0, 0, 0)  # Assuming robot starts at origin 
TURNING_RADIUS = 0.57  # Example turning radius in meters, adjust as needed

# ...existing code...

def create_path_dataframe(full_path, forward_vel, backward_vel):
    """
    Converts the list of path objects from Reeds-Shepp into a Pandas DataFrame
    and adds velocity and duration columns.
    Args:
        full_path: List of path objects (from generate_path_segment).
        forward_vel: Forward velocity (not used directly now, kept for compatibility).
        backward_vel: Backward velocity (not used directly now, kept for compatibility).
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

        # Add velocity column based on gear, steering, and distance
        # Assumes rs.Gear and rs.Steering enums are defined in reeds_shepp module
        def get_velocity(gear, steering, distance):
            is_short = distance < 0.3
            if gear == rs.Gear.FORWARD:
                if steering == rs.Steering.STRAIGHT:
                    return FORWARD_STRAIGHT_SHORT_VELOCITY if is_short else FORWARD_STRAIGHT_LONG_VELOCITY
                elif steering == rs.Steering.LEFT:
                    return FORWARD_LEFT_SHORT_VELOCITY if is_short else FORWARD_LEFT_LONG_VELOCITY
                elif steering == rs.Steering.RIGHT:
                    return FORWARD_RIGHT_SHORT_VELOCITY if is_short else FORWARD_RIGHT_LONG_VELOCITY
            elif gear == rs.Gear.BACKWARD:
                if steering == rs.Steering.STRAIGHT:
                    return BACKWARD_STRAIGHT_SHORT_VELOCITY if is_short else BACKWARD_STRAIGHT_LONG_VELOCITY
                elif steering == rs.Steering.LEFT:
                    return BACKWARD_LEFT_SHORT_VELOCITY if is_short else BACKWARD_LEFT_LONG_VELOCITY
                elif steering == rs.Steering.RIGHT:
                    return BACKWARD_RIGHT_SHORT_VELOCITY if is_short else BACKWARD_RIGHT_LONG_VELOCITY
            return 0  # Default for neutral or unknown

        df['velocity'] = df.apply(lambda row: get_velocity(row['gear'], row['steering'], row['distance']), axis=1)

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

# ...existing code...

# to send velocity over UART need to multiply by 100 and convert to int
velocity_int = int(row['velocity'] * 100)  # Zmiana: wysyłaj prędkość w cm/s jako dwucyfrową wartość int
            # ...existing code...