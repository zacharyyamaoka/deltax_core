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
deltax.sendGcode('G28') # Go Home
deltax.wait_for_robot_response()

# region - Testing Pick and Place pattern
# width = 500
# z_top = -750
# z_bottom = -900
# radius = 60

# deltax.sendGcode(f"G0 Y0 X{width/2}  Z{z_bottom} W90 U0 V-20  F500 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X{width/2}  Z{z_top} W0 U0 V-20  F500 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X{-width/2}  Z{z_top} W0 U0 V-20  F500 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X{-width/2}  Z{z_bottom} W-90 U0 V-20  F500 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X{-width/2}  Z{z_bottom} W-90 U25 V-20  F500 A500 S0 E0")
# deltax.sendGcode(f"G4 P2000")
# deltax.sendGcode(f"G0 Y0 X{-width/2}  Z{z_top} W0 U0 V-20  F500 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X{width/2}  Z{z_top} W0 U0 V-20  F500 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X{width/2}  Z{z_bottom} W90 U0 V-20  F500 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X{width/2}  Z{z_bottom} W90 U25 V-20  F500 A500 S0 E0")
# deltax.wait_for_robot_response()
#endregion

#region - Start and End Velocities

# # Slow Transition
# start_time = time.time()
# E = 0
# deltax.sendGcode("G0 Y0 X300  Z-800 W0 U0 V0 F3000 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X0  Z-800 W0 U0 V0  F3000 A500 S0 E{E}")
# deltax.sendGcode(f"G0 X0 Y0 Z-900 W0 U0 V0  F3000 A500 S{E} E0")
# deltax.wait_for_robot_response()

# print(time.time() - start_time)
# start_time = time.time()

# # Smooth Transition
# E = -100
# deltax.sendGcode("G0 Y0 X300  Z-800 W0 U0 V0 F3000 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X0  Z-800 W0 U0 V0  F3000 A500 S0 E{E}")
# deltax.sendGcode(f"G0 X0 Y0 Z-900 W0 U0 V0  F3000 A500 S{E} E0")
# deltax.wait_for_robot_response()

# print(time.time() - start_time)
# start_time = time.time()
# E = -250
# deltax.sendGcode("G0 Y0 X300  Z-800 W0 U0 V0 F3000 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X0  Z-800 W0 U0 V0  F3000 A500 S0 E{E}")
# deltax.sendGcode(f"G0 X0 Y0 Z-900 W0 U0 V0  F3000 A500 S{E} E0")
# deltax.wait_for_robot_response()
# print(time.time() - start_time)
# start_time = time.time()

# deltax.sendGcode("G0 Y0 X300  Z-800 W0 U0 V0 F3000 A500 S0 E0")
# deltax.sendGcode(f"G0 Y0 X0  Z-800 W0 U0 V0  F3000 A500 S-500 E-250")
# deltax.sendGcode(f"G0 X0 Y0 Z-900 W0 U0 V0  F3000 A500 S-250 E0")
# deltax.wait_for_robot_response()
# print(time.time() - start_time)
start_time = time.time()


deltax.sendGcode("G0 Y0 X300  Z-800 W0 U0 V0 F3000 A500 S0 E0")
deltax.sendGcode(f"G0 Y0 X0  Z-800 W0 U0 V0  F3000 A500 S500 E500")
# deltax.sendGcode(f"G0 X0 Y0 Z-850 W0 U0 V0  F3000 A500 S100 E0")
deltax.wait_for_robot_response()
print(time.time() - start_time)
start_time = time.time()
time.sleep(2)

# deltax.sendGcode("G0 Y0 X300  Z-800 W0 U0 V0 F3000 A20000 S0 E0")
# deltax.sendGcode(f"G0 Y0 X0  Z-800 W0 U0 V0  F3000 A20000 S-500 E-250")
# deltax.sendGcode(f"G0 X0 Y0 Z-900 W0 U0 V0  F3000 A20000 S-250 E0")
# deltax.wait_for_robot_response()
# print(time.time() - start_time)
# start_time = time.time()

# E = -100
# deltax.sendGcode("G0 Y0 X300  Z-800 W0 U0 V0 F3000 A20000 S0 E0")
# deltax.sendGcode(f"G0 Y0 X0  Z-800 W0 U0 V0  F3000 A20000 S0 E{E}")
# deltax.sendGcode(f"G0 X0 Y0  Z-900 W0 U0 V0  F3000 A20000 S{E} E0")
# deltax.wait_for_robot_response()

#endregion

# region Check XYZ acceleration motion
# max speed is 3 m/s, max accel is 50m/s^2

# for i in range(0,20):
    
#     start_time = time.time()

#     F = 3000
#     A = i*2000 + 500
#     deltax.sendGcode(f"G0 Y0 X-390  Z-800 W0 U0 V0  F{F} A{A} J910000")
#     deltax.sendGcode(f"G0 Y0 X390  Z-800 W0 U0 V0  F{F} A{A} J910000")
#     deltax.wait_for_robot_response()

#     iteration_time = time.time() - start_time
#     print(f"Iteration {i} took {iteration_time:.6f} seconds")

#endregion


deltax.sendGcode('G28') # Go Home

print("Finished Warm Up")