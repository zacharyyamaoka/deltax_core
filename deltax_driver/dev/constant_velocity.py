from deltax_driver.deltax_ros import DeltaX
import time
import sys
import rclpy

rclpy.init()
node = rclpy.create_node('send_gcode')

deltax = DeltaX(port="/dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00")

if deltax.connect():
    print("Connected - Opening Terminal")
else:
    print("Couldn't Connect")
    sys.exit(1)

# deltax.sendGcode("Emergency:Reset")
deltax.sendGcode('M210 F3000 A500 S0 E0')
deltax.sendGcode('M211 F360 A1000 S0 E0')
deltax.sendGcode('M212 F200 A1000 S0 E0')
deltax.sendGcode('M213 F100 A1000 S0 E0')
deltax.sendGcode(f"M220 I0") # XYZ
deltax.sendGcode(f"M220 I1") # axis 4
deltax.sendGcode(f"M220 I2") # axis 5
deltax.sendGcode(f"M220 I3") # axis 6

# Starting Code
deltax.sendGcode('M100 A1 B10') # Stream position (A = 1) at 10 ms (B=10)
# deltax.sendGcode('G28') # Go Home
deltax.wait_for_robot_response()

# region - Follow Belt
# for i in range(100):
#     deltax.sendGcode('G0 X0 Y300 Z-900 F3000 A5000') # Stream position (A = 1) at 10 ms (B=10)
#     deltax.sendGcode('G0 X0 Y-300 Z-900 F100 A5000') # Stream position (A = 1) at 10 ms (B=10)
#     deltax.wait_for_robot_response()
# endregion - Follow Belt


for i in range(100):
    deltax.sendGcode('G0 X0 Y250 Z-870 F3000 A5000') # Stream position (A = 1) at 10 ms (B=10)
    deltax.sendGcode('G0 X0 Y200 Z-920 F100 A5000') # Stream position (A = 1) at 10 ms (B=10)
    deltax.wait_for_robot_response()

print("Finished Warm Up")