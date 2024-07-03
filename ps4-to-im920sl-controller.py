import pygame
import serial
import time

def map_axis_to_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    return int(((value - old_min) * new_range) / old_range + new_min)

# Set up serial communication with IM920SL
im920sl_ser = serial.Serial('/dev/ttyUSB0', '19200', timeout=None)

# Initialize Pygame
pygame.init()

# Initialize joystick
pygame.joystick.init()

# Get the number of joysticks
joystick_count = pygame.joystick.get_count()

if joystick_count > 0:
    # Get the joystick instance
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Display joystick name
    print(f"Joystick name: {joystick.get_name()}")

    # Time of last transmission
    last_send_time = time.time()

    while True:
        # Process events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # Get button states
        buttons = sum(joystick.get_button(i) << i for i in range(joystick.get_numbuttons()))
        buttons_hex = f"{buttons:04X}"

        # Get joystick axis states and convert to 0-255 range
        axes = [map_axis_to_range(joystick.get_axis(i), -1, 1, 0, 255) for i in range(joystick.get_numaxes())]
        axes_hex = "".join(f"{axis:02X}" for axis in axes)

        # Get d-pad (hat) state
        hat = joystick.get_hat(0)
        hats_hex = {
            (-1, -1): "0000", (0, -1): "8000", (1, -1): "FF00",
            (-1,  0): "0080", (0,  0): "8080", (1,  0): "FF80",
            (-1,  1): "00FF", (0,  1): "80FF", (1,  1): "FFFF"
        }.get(hat, "8080")  # Default is center position

        # Combine and display
        controller_data = buttons_hex + axes_hex + hats_hex
        # print(controller_data)

        # Send data every 0.052 seconds
        current_time = time.time()
        if current_time - last_send_time >= 0.052:
            if controller_data is not None:
                im920sl_ser.write(f'TXDA{controller_data}\r'.encode('utf-8'))
                print(controller_data)

            # Receive response from IM920SL
            im920sl_response = im920sl_ser.readline().decode('utf-8').rstrip()
            print(f"Response from IM920SL: {im920sl_response}")

            # Clear the last input data
            controller_data = None

            last_send_time = current_time

else:
    print("No joysticks found.")

# Quit Pygame
pygame.quit()
