from deltax_driver.robot import DeltaX
import threading
import time

""" 
    OBSOLETE, SEND FROM TERMINAL INSTEAD!
"""
def monitor_deltax_response(deltax):
    """Continuously monitor and print the most recent response from DeltaX."""
    while True:
        if deltax.isResponded():
            response = deltax.robot_response()
            if response:  # Ensure response is not empty
                print(f"DeltaX Response: {response}")
        time.sleep(0.1)  # Prevent CPU overload with a short delay

def main():

    deltax = DeltaX(port="/dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00")

    if deltax.connect():
        print("Connected - Opening Terminal")
        deltax.sendGcode('M210 F3000 A500 S0 E0')
        deltax.sendGcode('M211 F360 A1000 S0 E0')
        deltax.sendGcode('M212 F200 A1000 S0 E0')
        deltax.sendGcode('M213 F100 A1000 S0 E0')

        deltax.sendGcode('M100 A1 B10')
        deltax.sendGcode('Position')
        deltax.sendGcode('G28')
    else:
        print("Couldn't Connect")
        return

    last_command = ""  # Variable to store the last command

    while True:
        if not deltax.isResponded():  # If there are still commands to execute, then just wait
            time.sleep(0.01)
            continue

        try:
            # Get user input
            cmd = input("Enter G-code command: ")

            # Handle up arrow for recalling the last command
            if cmd == '\x1b[A':  # This is the code for up arrow (ESC + [ + A)
                if last_command:
                    print(f"Recalling last command: {last_command}")
                    cmd = last_command
                else:
                    print("No previous command to recall.")
                    continue

            last_command = cmd  # Update last command
            deltax.sendGcode(cmd)

        except KeyboardInterrupt:
            print("Exiting due to user interruption.")
            break

if __name__ == "__main__":
    main()

# G91 - Relative Movement Mode
# G90 - Absolute Movement Mode
