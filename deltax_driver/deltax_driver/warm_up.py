from deltax_driver.robot import DeltaX
import time
import sys

deltax = DeltaX(port="/dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00")

if deltax.connect():
    print("Connected - Opening Terminal")
else:
    print("Couldn't Connect")
    sys.exit(1)

deltax.sendGcode('M100 A1 B10')
deltax.sendGcode('G28')
deltax.wait_for_robot_response()

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
deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500")
time.sleep(2)
deltax.sendGcode("Emergency:Stop") # It seems to clear all outstanding commands
deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500") # rejected as Delta is Stopped
time.sleep(1)
deltax.sendGcode("Emergency:Resume")
deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500") # rejected as Delta is Stopped
deltax.wait_for_robot_response()

deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F50 A50")
time.sleep(2)
deltax.sendGcode("Emergency:Pause") 
time.sleep(1)
deltax.sendGcode("Emergency:Resume")
# deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500")
deltax.wait_for_robot_response()

# what happens when I send multiple g codes --> it waits until I have recivied 4 oks
deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F500 A500")
deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500")
deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F500 A500")
deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500")
deltax.wait_for_robot_response()

# while True:
#     deltax.sendGcode("G0 X0 Y-300  Z-800 W0 U0 V0  F500 A500")

#     deltax.wait_for_robot_response()
#     print('done 1')

#     deltax.sendGcode("G0 X0 Y300  Z-800 W0 U0 V0  F500 A500")

#     deltax.wait_for_robot_response()

#     print('done 2')

print("Finished Warm Up")