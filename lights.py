# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
# Wait a heartbeat before sending commands
master.wait_heartbeat()


# Arm
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     1, 0, 0, 0, 0, 0, 0)

# To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
# It's possible to check and configure this buttons in the Joystick menu of QGC
# buttons = 1 + 1 << 3 + 1 << 7
buttons = 1 << 9
# buttons = 1 << 15
print('{0:b}'.format(buttons))
master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    0,
    0,
    buttons)
