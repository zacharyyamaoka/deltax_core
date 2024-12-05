from deltax_driver.robot import DeltaX
import time
import sys

deltax = DeltaX(port="/dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00")

if deltax.connect():
    print("Connected - Opening Terminal")
else:
    print("Couldn't Connect")
    sys.exit(1)

deltax.sendGcode('M211 F360 A500 S0 E0')
deltax.sendGcode('M212 F360 A500 S0 E0')
deltax.sendGcode('M213 F360 A500 S0 E0')

# deltax.sendGcode('M100 A1 B10')
# deltax.sendGcode('G28')
# deltax.wait_for_robot_response()

deltax.sendGcode("IsDelta")
deltax.wait_for_robot_response()

deltax.sendGcode("Position")
deltax.wait_for_robot_response()

for i in range(1,20):
    
    start_time = time.time()

    F = max(100 + i*100,3000)
    A = 100 + i*100
    deltax.sendGcode(f"G0 X0 Y0 Z-800 W-{i*10}")
    deltax.sendGcode(f"G0 X0 Y0 Z-800 W{i*10}")
    deltax.wait_for_robot_response()

    iteration_time = time.time() - start_time
    print(f"Iteration {i} took {iteration_time:.6f} seconds")

print("Finished Test")