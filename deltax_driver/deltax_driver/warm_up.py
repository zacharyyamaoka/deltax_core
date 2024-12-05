from deltax_driver.robot import DeltaX
import time
import sys

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

#region - Useful Commands
deltax.sendGcode("IsDelta")
deltax.wait_for_robot_response()

deltax.sendGcode("Position")
deltax.wait_for_robot_response()

deltax.sendGcode("PositionOffset")
deltax.wait_for_robot_response()

deltax.sendGcode("Angle")
deltax.wait_for_robot_response()

deltax.sendGcode("HTs")
deltax.wait_for_robot_response()

deltax.sendGcode("IMEI")
deltax.wait_for_robot_response()

deltax.sendGcode("Infor")
deltax.wait_for_robot_response()

# Unknown Commands
deltax.sendGcode("DeltaState")
deltax.sendGcode("TempState")
deltax.wait_for_robot_response()

#endregion

# region - Testing Emergency Stop
deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500") # Go to start
deltax.wait_for_robot_response()

deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F50 A50")
deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F500 A500")
time.sleep(2)
deltax.sendGcode("Emergency:Stop") # It seems to clear all outstanding commands
deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F500 A500") # rejected as Delta is Stopped
time.sleep(1)
deltax.sendGcode("Emergency:Resume")
deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F500 A500") # Reach End
deltax.wait_for_robot_response()

deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F50 A50") # Go to start
time.sleep(2)
deltax.sendGcode("Emergency:Pause") 
time.sleep(1)
deltax.sendGcode("Emergency:Resume")
deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500") # Reach end
deltax.wait_for_robot_response()

#endregion

#region - Sending trajectory
deltax.sendGcode("G0 X0 Y0  Z-920 W0 U0 V-20  F500 A500 S0 E0")
deltax.sendGcode("G0 X0 Y-300  Z-780 W0 U0 V0  F500 A500 S0 E0")
deltax.sendGcode("G0 X0 Y300  Z-920 W0 U0 V0  F500 A500 S0 E0")
deltax.sendGcode("G0 X0 Y-300  Z-780 W0 U0 V0  F500 A500 S0 E0")
deltax.wait_for_robot_response()
#endregion

# region - Testing Pick and Place pattern
width = 500
z_top = -800
z_bottom = -950
radius = 60

deltax.sendGcode(f"G0 X0 Y{width/2}  Z{z_bottom} W90 U0 V-20  F500 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y{width/2}  Z{z_top} W0 U0 V-20  F500 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y{-width/2}  Z{z_top} W0 U0 V-20  F500 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y{-width/2}  Z{z_bottom} W-90 U0 V-20  F500 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y{-width/2}  Z{z_bottom} W-90 U25 V-20  F500 A500 S0 E0")
deltax.sendGcode(f"G4 P2000")
deltax.sendGcode(f"G0 X0 Y{-width/2}  Z{z_top} W0 U0 V-20  F500 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y{width/2}  Z{z_top} W0 U0 V-20  F500 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y{width/2}  Z{z_bottom} W90 U0 V-20  F500 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y{width/2}  Z{z_bottom} W90 U25 V-20  F500 A500 S0 E0")
deltax.wait_for_robot_response()
#endregion

#region- TODO Doesn't work, the angles jam up, Delta X is looking into it
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U0 V0")
# deltax.sendGcode(f"G6 X45 Y0 Z0 W0 U0 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U0 V0")
# deltax.sendGcode(f"G6 X0 Y45 Z0 W0 U0 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U0 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z45 W0 U0 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U0 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W20 U0 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U0 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U45 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U0 V0")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U0 V45")
# deltax.sendGcode(f"G6 X0 Y0 Z0 W0 U0 V0")
# deltax.wait_for_robot_response()
#endregion 

#region - Linear Joint Jog
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.sendGcode("G0 X-100 Y0  Z-800 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.sendGcode("G0 X0 Y-100  Z-800 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0  Z-900 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0  Z-800 W-45 U0 V0")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U-45 V0")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V45")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.wait_for_robot_response()
#endregion

# region - Testing setting new joint limit
deltax.sendGcode("M62 H0 P30 Q-45")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V45")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.sendGcode("M62 H0 P90 Q-20")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V45")
deltax.sendGcode("G0 X0 Y0  Z-800 W0 U0 V0")
deltax.wait_for_robot_response()
# endregion

# region - Testing absolute vs relative mode
deltax.sendGcode("G0 X0 Y0 Z-800 W0 U0 V0")
deltax.sendGcode("G91")
deltax.sendGcode("G0 X0 Y0 Z-10 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0 Z-10 W0 U0 V0")
deltax.sendGcode("G93")
deltax.sendGcode("G0 X0 Y0 Z10 W0 U0 V0")
deltax.sendGcode("G0 X0 Y0 Z10 W0 U0 V0")
deltax.sendGcode("G93")

deltax.sendGcode("G90")
deltax.sendGcode("G0 X0 Y0 Z-900 W0 U0 V0")
deltax.sendGcode("G93")
deltax.wait_for_robot_response()

#endregion

#region - Start and End Velocities

# Slow Transition
E = 0
deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0 F3000 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y0  Z-800 W0 U0 V0  F3000 A500 S0 E{E}")
deltax.sendGcode(f"G0 X-200 Y-250  Z-780 W0 U0 V0  F3000 A500 S{E} E0")
deltax.wait_for_robot_response()

# Smooth Transition
E = -500
deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0 F3000 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y0  Z-800 W0 U0 V0  F3000 A500 S0 E{E}")
deltax.sendGcode(f"G0 X-200 Y-250  Z-780 W0 U0 V0  F3000 A500 S{E} E0")
deltax.wait_for_robot_response()

# Sudden vs Smooth stop
E = -500
deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0 F3000 A500 S0 E0")
deltax.sendGcode(f"G0 X0 Y0  Z-800 W0 U0 V0  F3000 A500 S0 E{E}")
deltax.sendGcode(f"G0 X-200 Y-250  Z-780 W0 U0 V0  F3000 A500 S0 E0")

deltax.wait_for_robot_response()

#endregion

# region Check XYZ acceleration motion
# max speed is 3 m/s, max accel is 50m/s^2

for i in range(1,11):
    
    start_time = time.time()

    F = 3000
    A = i*1000
    deltax.sendGcode(f"G0 X0 Y-300  Z-800 W0 U0 V0  F{F} A{A}")
    deltax.sendGcode(f"G0 X0 Y300  Z-800 W0 U0 V0  F{F} A{A}")
    deltax.wait_for_robot_response()

    iteration_time = time.time() - start_time
    print(f"Iteration {i} took {iteration_time:.6f} seconds")

#endregion

# region Check Circular acceleration motion
deltax.sendGcode(f"G0 X0 Y300  Z-800 W0 U0 V0 A500")

for i in range(1,5):
    
    start_time = time.time()

    F = 3000
    A = i*1000
    E = -1000
    deltax.sendGcode(f"G2 I0 J-300 X0 Y-300 S0 E0 F{F} A{A}")
    deltax.sendGcode(f"G2 I0 J300 X0 Y300 S0 E0 F{F} A{A}")
    deltax.sendGcode(f"G3 I0 J-300 X0 Y-300 S0 E{E} F{F} A{A}")
    deltax.sendGcode(f"G3 I0 J300 X0 Y300 S{E} E0 F{F} A{A}")
    deltax.wait_for_robot_response()

    iteration_time = time.time() - start_time
    print(f"Iteration {i} took {iteration_time:.6f} seconds")

#endregion

deltax.sendGcode('G28') # Go Home

print("Finished Warm Up")