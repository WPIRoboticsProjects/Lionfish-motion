# Import mavutil

from pymavlink import mavutil
import time
from signal import signal, SIGINT
from sys import exit

# Pwm channel pins
# 0 - pitch
# 1 - roll
# 2 - up
# 3 - yaw
# 4 - forward
# 5 - lateral
# 6 - camera pan
# 7 - camera tilt
# 8 - lights 1 level

def run(master):

    while(True):
        try:
            command = input("Command: ")
            print("Given command: " + command + "\n")
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
                elif verb == 13:
                    # turn to given angle
                    val = int(commands[1])  # throttle
                    rel_angle = float(commands[2])  # target angle
                    if val > 0 and val <= 100:
                        output = (val*5) + 1500
                        if rel_angle < 0:
                            output = (-val * 5) + 1500

                        org_heading = float(getMessage(master)['heading'])
                        curr_heading = org_heading
                        old_time = time.time()

                        while continue_turn(org_heading, curr_heading, rel_angle):
                            write_pwm(master, 3, output)
                            curr_heading = float(getMessage(master)['heading'])
                            new_time = time.time()
                            print(new_time - old_time)
                            old_time = new_time

                    elif val == 0:
                        write_pwm(master, 3, 0)
                elif verb == 14:
                    # drive forward
                    val = int(commands[1])
                    time_to_drive = float(commands[2])
                    if val > 0 and val <= 100:
                        output = (val * 5) + 1500
                        end_time = time.time() + time_to_drive
                        while time.time() < end_time:
                            write_pwm(master, 4, output)
                            time.sleep(0.05)

                elif verb == 15:
                    # drive backward
                    val = int(commands[1])
                    time_to_drive = float(commands[2])
                    if val > 0 and val <= 100:
                        output = (-val * 5) + 1500
                        end_time = time.time() + time_to_drive
                        while time.time() < end_time:
                            write_pwm(master, 4, output)
                            time.sleep(0.05)
                elif verb == 16:
                    # dive to given depth
                    val = int(commands[1])
                    target_depth = float(commands[2])
                    if val > 0 and val <= 100:
                        curr_depth = float(getMessage(master)['alt'])
                        output = (val*5) + 1500
                        if(target_depth - curr_depth) < 0:
                            output = (-val*5) + 1500

                        while abs(target_depth - curr_depth) > 0.2:
                            write_pwm(master, 2, output)
                            curr_depth = float(getMessage(master)['alt'])
                    elif val == 0:
                        write_pwm(master, 2, 0)
                elif verb == 100:
                    print(getMessage(master))
                else:
                    pass
            else:
                print("Unknown command, list of available commands: \n")
                print_cmd_list()
                print("")
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
    elif string_in == "reverse":
        return 15
    elif string_in == "dive":
        return 16
    elif string_in == "hud":
        return 100
    elif string_in == "quit":
        return -2
    elif string_in == "q":
        return -2
    else:
        return -1

def write_pwm(master, output_channel, output_val):
    rc_channel_values = [65535 for _ in range(8)]
    rc_channel_values[output_channel] = output_val  # vertical
    master.mav.rc_channels_override_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        *rc_channel_values)

def continue_turn(org_heading, curr_heading, rel_angle):
    if get_quad(curr_heading) == get_quad(org_heading + rel_angle):
        error = abs((org_heading + rel_angle) - curr_heading)
        print(error)
        if error < 5:
            return False
        else:
            return True
    else:
        return True

def get_quad(angle):
    if angle >= 0 and angle < 90:
        return 0
    elif angle >= 90 and angle < 180:
        return 1
    elif angle >= 180 and angle < 270:
        return 2
    elif angle >= 270 and angle <= 360:
        return 3

def getMessage(master):
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        # print(msg.get_type())
        if msg.get_type() == 'VFR_HUD':
            # print("\n\n*****Got message: %s*****" % msg.get_type())
            # print("Message: %s" % msg)
            # print("\nAs dictionary: %s" % msg.to_dict())
            return msg.to_dict()

def print_cmd_list():
    print("arm - arm the motors")
    print("disarm - disarm the motors")
    print("depth - depth mode")
    print("stab - stabilize mode")
    print("man - manual mode")
    print("lights - toggle lights")
    print("hold - hold last sent command")
    print("camdown - move camera down")
    print("camup - move camera up")
    print("yaw <0-100% throttle> <relative degrees> - turn robot")
    print("forward <0-100% throttle> <time in seconds> - drive forward for x seconds")
    print("reverse <0-100% throttle> <time in seconds> - drive reverse for x seconds")
    print("dive <0-100% throttle> <target depth (m)> - dive to given depth")
    print("hud - print out the hud data")
    print("q - quit the program")


def main():
    # Create the connection
    master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    run(master)


if __name__ == '__main__':
    # signal(SIGINT, handler)
    main()