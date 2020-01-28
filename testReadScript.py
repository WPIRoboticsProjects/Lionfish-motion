# # import time
# # # Import mavutil
# # from pymavlink import mavutil
# #
# # # Create the connection
# # #  If using a companion computer
# # #  the default connection is available
# # #  at ip 192.168.2.1 and the port 14550
# # # Note: The connection is done with 'udpin' and not 'udpout'.
# # #  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
# # #  uses a 'udpbcast' (client) and not 'udpin' (server).
# # #  If you want to use QGroundControl in parallel with your python script,
# # #  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
# # #  E.g: --out udpbcast:192.168.2.255:yourport
# # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# #
# # # Get some information !
# # while True:
# #     try:
# #         print(master.recv_match().to_dict())
# #     except:
# #         pass
# #     time.sleep(0.1)
#
# # Import mavutil
# from pymavlink import mavutil
# import time
#
# # Create the connection
# master = mavutil.mavlink_connection('udp:0.0.0.0:15000')
# # Wait a heartbeat before sending commands
# master.wait_heartbeat()
#
# # Request all parameters
# master.mav.param_request_list_send(
#     master.target_system, master.target_component
# )
# while True:
#     time.sleep(0.01)
#     try:
#         message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
#         print('name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
#     except Exception as e:
#         print(e)
#         exit(0)

# Import mavutil
from pymavlink import mavutil

# Create the connection
# From topside computer
master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')

while True:
    msg = master.recv_match()
    if not msg:
        continue
    # print(msg.get_type())
    if msg.get_type() == 'VFR_HUD':
        print("\n\n*****Got message: %s*****" % msg.get_type())
        print("Message: %s" % msg)
        print("\nAs dictionary: %s" % msg.to_dict())
    # if msg.get_type() == 'HEARTBEAT':
    #     print("\n\n*****Got message: %s*****" % msg.get_type())
    #     print("Message: %s" % msg)
    #     print("\nAs dictionary: %s" % msg.to_dict())
    #     # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
    #     print("\nSystem status: %s" % msg.system_status)