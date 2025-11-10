def set_flags():
	try:
	for index, row in path_df.iterrows():
		if row['steering']   == rs.Steering.LEFT:  steering_char = 'R'
                elif row['steering'] == rs.Steering.RIGHT: steering_char = 'L'
                else: steering_char = 'N'

            	gear_char = 'F' if row['gear'] == rs.Gear.FORWARD else ('B' if row['gear'] == rs.Gear.BACKWARD else 'N')

		…

while not end of DataFrame

response_line = uart_comm.serial_port.readline().decode('ascii', errors='ignore').strip()

if response_line == "1":
	set_flags()
	uart_comm.send_command(
        steering=steering_char,
        gear=gear_char,
	duration=duration_ms_int,
	lifting=lifting
        )
else: break

