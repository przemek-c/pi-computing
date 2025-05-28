import serial
import time

class UARTCommunication:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

    def send_command(self, steering, gear, velocity, use_controller):
        """
        Send a formatted command message  
        steering: 'L' or 'R' (Left/Right)  
        gear: 'F' or 'B' (Forward/Backward)  
        not sure if I need move_type: 'A' or 'D' (Acceleration/Deceleration)  
        velocity: int (speed value)  
        don't need duration either I think: int (time in milliseconds)  
        """
        # Added an extra '[' at the start to ensure it's present
        message = f"[[S:{steering},G:{gear},V:{velocity},C:{use_controller}]\n"
        print(f"Sending UART message: {message.strip()}")  # Print to terminal
        
        # Ensure proper encoding and that the message starts with '['
        encoded_message = message.encode('ascii')

        self.serial_port.write(encoded_message)
        
        # Flush the output buffer to ensure all data is sent
        self.serial_port.flush()
        time.sleep(0.1)  # Small delay to ensure message is sent

    def close(self):
        self.serial_port.close()

def main():
    # For Windows, use 'COM3' (or appropriate COM port)
    # For Linux/Raspberry Pi, use '/dev/ttyUSB0' or '/dev/ttyAMA0'
    uart = UARTCommunication(port='/dev/ttyAMA0')    
    try:
        # Example: Turn right, forward gear, accelerate, velocity 50, duration 2000ms
        uart.send_command(
            steering='R',    # Right turn
            gear='F',        # Forward
            # move_type='A',   # Acceleration not sure if needed
            velocity=10,     # Speed value
            # duration=2,       # 2s duration
            use_controller=1  # Use controller
        )

        # Output: [S:R,G:F,V:50,D:2]
        
    finally:
        uart.close()

if __name__ == "__main__":
    main()