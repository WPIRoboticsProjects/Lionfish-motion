from pymavlink import mavutil
from multiprocessing import Process, Queue
import time
from sys import exit
import serial
import signal

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

TURN_BUFFER = 2

startMarker = 60
endMarker = 62

def handler(signum, frame):
    print('Handle Ctrl-C')
    handle_exit()
    exit()

def handle_exit():
    print("Exiting")
    exit()
    

def run(master, qFromArduino, qToArduino):
    main_loop_queue = Queue()
    main_loop_process = Process(target=main_loop, args=(master, main_loop_queue, qFromArduino, qToArduino,))
    #main_loop_process.daemon = True
    main_loop_process.start()

    cont_run = True
    while cont_run:
        command = input("Command: ")
        main_loop_queue.put(command)
        
        commands = command.split()
        verb = lookup_button(commands[0])
        if verb == -2:
            handle_exit()
        time.sleep(1)


def main_loop(master, main_loop_queue, qFromArduino, qToArduino):
    forward_ping = -100
    down_ping = -100
    ping1, ping2 = update_sensors(qFromArduino)
    if ping1 != -100:
        forward_ping = ping1
    if ping2 != -100:
        down_ping = ping2

    cont_run = True
    while cont_run:
        try:

            if not main_loop_queue.empty():
                command = main_loop_queue.get()
                # command = input("Command: ")
                print("Given command: " + command + "\n")
                commands = command.split()
                verb = lookup_button(commands[0])
                if verb == -2:
                    handle_exit()

                if verb == 101:
                    if commands[1] == 1:
                        print(forward_ping)
                    elif commands[1] == 2:
                        print(down_ping)
                elif verb != -1:
                    motor_cmd_process = Process(target=motor_cmd, args=(master, verb, commands))
                    motor_cmd_process.daemon = True
                    motor_cmd_process.start()
                else:
                    print("Unknown command, list of available commands: \n")
                    print_cmd_list()
                    print("")

            # update sensors
            ping1, ping2 = update_sensors(qFromArduino)
            if ping1 != -100:
                forward_ping = ping1
            if ping2 != -100:
                down_ping = ping2
            #print("forward: %f, down: %f" % (forward_ping,down_ping))

        except Exception as e:
            print("Incorrect command: " + str(e))
            exit()

def update_sensors(q):

    ping1_val = -100
    ping2_val = -100
    if not q.empty():
        val = q.get()
        if val[0] == 0:
            ping1_val = val[1]
        elif val[0] == 1:
            ping2_val = val[1]

        for i in range(q.qsize()):
            val = q.get()
            if val[0] == 0:
                ping1_val = val[1]
            elif val[0] == 1:
                ping2_val = val[1]
        #print("ping1: %f, ping2: %f" % (ping1_val, ping2_val))
    return ping1_val, ping2_val


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
    elif string_in == "square":
        return 17
    elif string_in == "hud":
        return 100
    elif string_in == "ping":
        return 101
    elif string_in == "quit":
        return -2
    elif string_in == "q":
        return -2
    else:
        return -1


def motor_cmd(master, verb, commands):
    if verb < 13:
        button_press(master, verb)
    elif verb == 13:
        # turn to given angle
        val = int(commands[1])  # throttle
        rel_angle = float(commands[2])  # target angle
        turn_angle(master, val, rel_angle)
    elif verb == 14:
        # drive forward
        val = int(commands[1])
        time_to_drive = float(commands[2])
        drive_forward(master, val, time_to_drive)
    elif verb == 15:
        # drive backward
        val = int(commands[1])
        time_to_drive = float(commands[2])
        drive_backward(master, val, time_to_drive)
    elif verb == 16:
        # dive to given depth
        val = int(commands[1])
        target_depth = float(commands[2])
        depth(master, val, target_depth)
    elif verb == 17:
        # run square
        drive_forward(master, 50, 8)
        clear_motors(master)
        time.sleep(1)

        turn_angle(master, 15, 90)
        drive_forward(master, 50, 4)
        clear_motors(master)
        time.sleep(1)

        turn_angle(master, 15, 90)
        drive_forward(master, 50, 8)
        clear_motors(master)
        time.sleep(1)

        turn_angle(master, 15, 90)
        drive_forward(master, 50, 4)
        clear_motors(master)
        time.sleep(1)

        turn_angle(master, 15, 90)

    elif verb == 100:
        print(get_message(master))
    else:
        pass


def turn_angle(master, val, rel_angle):
    if val > 0 and val <= 100:
        output = (val * 5) + 1500
        if rel_angle < 0:
            output = (-val * 5) + 1500

        org_heading = float(get_message(master)['heading'])
        curr_heading = org_heading

        while continue_turn(org_heading, curr_heading, rel_angle):
            write_pwm(master, 3, output)
            curr_heading = float(get_message(master)['heading'])

    elif val == 0:
        write_pwm(master, 3, 0)


def drive_forward(master, val, time_to_drive):
    if val > 0 and val <= 100:
        output = (val * 5) + 1500
        end_time = time.time() + time_to_drive
        while time.time() < end_time:
            write_pwm(master, 4, output)


def drive_backward(master, val, time_to_drive):
    if val > 0 and val <= 100:
        output = (-val * 5) + 1500
        end_time = time.time() + time_to_drive
        while time.time() < end_time:
            write_pwm(master, 4, output)


def depth(master, val, target_depth):
    if val > 0 and val <= 100:
        curr_depth = float(get_message(master)['alt'])
        output = (val * 5) + 1500
        if (target_depth - curr_depth) < 0:
            output = (-val * 5) + 1500

        while abs(target_depth - curr_depth) > 0.2:
            write_pwm(master, 2, output)
            curr_depth = float(get_message(master)['alt'])
    elif val == 0:
        write_pwm(master, 2, 0)


def button_press(master, verb):
    buttons = 1 << verb
    master.mav.manual_control_send(
        master.target_system,
        0,
        0,
        0,
        0,
        buttons)


def clear_motors(master):
    rc_channel_values = [0 for _ in range(8)]
    master.mav.rc_channels_override_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        *rc_channel_values)


def write_pwm(master, output_channel, output_val):
    rc_channel_values = [65535 for _ in range(8)]
    rc_channel_values[output_channel] = output_val
    master.mav.rc_channels_override_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        *rc_channel_values)


def continue_turn(org_heading, curr_heading, rel_angle):

    final_heading = org_heading + rel_angle
    if final_heading > 360:
        final_heading -= 360
    if final_heading < 0:
        final_heading += 360

    if (final_heading + TURN_BUFFER) > 360:
        if abs(final_heading - 360 - curr_heading) < TURN_BUFFER:
            return False
    if (final_heading - TURN_BUFFER) < 0:
        if abs(final_heading + 360 - curr_heading) < TURN_BUFFER:
            return False
    if abs(final_heading - curr_heading) < TURN_BUFFER:
        return False
    else:
        return True


def get_message(master):
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
    print("ping <ID> - return ping data from given ID, start at 1")
    print("square - run a rectangle")
    print("q - quit the program")


# Adruino -------------------------------------------------------------------


def recv_from_arduino(ser):
    global startMarker, endMarker

    ck = ""
    x = "z"  # any value that is not an end- or startMarker
    byteCount = -1  # to allow for the fact that the last increment will be one too many

    # wait for the start character
    while ord(x) != startMarker:
        x = ser.read()

    # save data until the end marker is found
    while ord(x) != endMarker:
        if ord(x) != startMarker:
            ck = ck + x.decode("utf-8")
            byteCount += 1
        x = ser.read()

    return (ck)


def arduino_comms(qToArduino, qFromArduino):
    ser = serial.Serial("/dev/serial/by-path/platform-70090000.xusb-usb-0:2.3:1.0", 115200, timeout=0)

    while True:
        if ser.inWaiting() > 0:
            try:
                dataRecvd = recv_from_arduino(ser)
                #print("Reply Received  " + dataRecvd)
                process_arduino_data(dataRecvd, qFromArduino)
            except:
                # print("cannot read")
                pass


def process_arduino_data(message, qFromArduino):
    recvMessage = message.split()
    messType = int(recvMessage[1])
    messId = int(recvMessage[2])
    messData = int(recvMessage[3])
    #print("Type: " + str(messType) + ", id: " + str(messId) + ", data: " + str(messData))
    if messType == 0:
        if messId == 1:
            qFromArduino.put((0, messData))
        elif messId == 2:
            qFromArduino.put((1, messData))
        # ping sensor update
        # qFromArduino # send received data to jetson
    elif messType == 1:
        pass
        # spear move update


# def actuate_spear(send_input):
#     if type(input) == 'int':
#         ser.write(("<" + str(send_input) + ">").encode('utf-8'))
#     else:
#         print("incorrect data type sent to spear")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    qToArduino = Queue()
    qFromArduino = Queue()

    # Create the connection
    master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    arduinoProcess = Process(target=arduino_comms, args=(qToArduino, qFromArduino,))
    arduinoProcess.daemon = True
    arduinoProcess.start()

    run(master, qFromArduino, qToArduino)
