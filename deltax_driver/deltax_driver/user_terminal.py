from deltax_driver.robot import DeltaX
import threading
import time

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
    else:
        print("Couldn't Connect")
        return


    while True:

        if deltax.isResponded() == False: # If there are still commands to execute then just wait
            # print(deltax.robot_response())
            time.sleep(0.01) 
            continue

        try:
            # Get user input
            cmd = input("Enter G-code command: ")

            deltax.sendGcode(cmd)

        except KeyboardInterrupt:
            print("Exiting due to user interruption.")
            break

if __name__ == "__main__":
    main()
