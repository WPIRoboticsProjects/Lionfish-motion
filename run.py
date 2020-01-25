# Import mavutil
from pymavlink import mavutil
import time
from signal import signal, SIGINT
from sys import exit

# def handler(signal_received, frame):
#     # Handle any cleanup here
#     print('SIGINT or CTRL-C detected. Exiting gracefully')
#     exit()

def run(master):

    while(True):
        try:
            command = input("Command: ")
            print("Given command: " + command)
            commands = command.split()
            verb = lookup_button(commands[0])
            if verb == -2:
                exit()

            if verb != -1:
                if verb < 13:
                    buttons = 1 << verb
                    master.mav.manual_control_send(
                        master.target_system,
                        0,
                        0,
                        0,
                        0,
                        buttons)
                else:

                    val = int(commands[1])
                    if val <= 1000 and val >= -1000:
                        buttons = 1 << 10
                        i = 0
                        while(i < 3000):
                            master.mav.manual_control_send(
                                master.target_system,
                                0,
                                0,
                                0,
                                val,
                                buttons)
                            time.sleep(0.001)
                            i += 1
                        master.mav.manual_control_send(
                            master.target_system,
                            0,
                            0,
                            0,
                            0,
                            buttons)
                    # master.mav.command_long_send(
                    #     master.target_system,
                    #     master.target_component,
                    #     mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
                    #     0,
                    #     45, 0.1, 1, 0, 0, 0, 0)
                    # master.mav.command_long_send(
                    #     master.target_system,
                    #     master.target_component,
                    #     mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    #     0,
                    #     90, 0.1, -1, 1, 0, 0, 0)
                    # while True:
                    #     master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
                    #     time.sleep(0.1)
        except Exception as e:
            print("Incorrect command: " + str(e))
            exit()

def lookup_button(string_in):
    if string_in == "depth":
        return 0
    elif string_in == "stab":
        return 1
    elif string_in == "man":
        return 2
    elif string_in == "disarm":
        return 4
    elif string_in == "arm":
        return 6
    elif string_in == "lights":
        return 9
    elif string_in == "hold":
        return 10
    elif string_in == "camdown":
        return 11
    elif string_in == "camup":
        return 12
    elif string_in == "yaw":
        return 13
    elif string_in == "forward":
        return 14
    elif string_in == "test":
        return 15
    elif string_in == "quit":
        return -2
    else:
        return -1

def main():
    # Create the connection
    master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    print(master.target_component)
    print(mavutil.mavlink.MAV_CMD_CONDITION_YAW)
    run(master)


if __name__ == '__main__':
    # signal(SIGINT, handler)
    main()