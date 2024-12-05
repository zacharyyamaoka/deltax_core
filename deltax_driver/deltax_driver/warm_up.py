from deltax_driver.robot import DeltaX
import time
import sys

deltax = DeltaX(port="/dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00")

if deltax.connect():
    print("Connected - Opening Terminal")
else:
    print("Couldn't Connect")
    sys.exit(1)


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

deltax.sendGcode("DeltaState")
deltax.wait_for_robot_response()

deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500")
deltax.wait_for_robot_response()

deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F50 A50")
time.sleep(2)
deltax.sendGcode("Emergency:Stop")
# deltax.wait_for_robot_response()

# deltax.sendGcode("Emergency:Resume")
# deltax.wait_for_robot_response()

# deltax.sendGcode("Emergency:Pause")
# deltax.wait_for_robot_response()
time.sleep(2)

deltax.sendGcode("Emergency:Resume")
deltax.wait_for_robot_response()

# while True:
#     deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500")

#     deltax.wait_for_robot_response()
#     print('done 1')

#     deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F500 A500")

#     deltax.wait_for_robot_response()

#     print('done 2')

print("Finished Warm Up")