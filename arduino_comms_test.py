# import serial
# import time
#
# ser = serial.Serial('COM3', 112500)
# ser.write(str.encode('c'))
#
#
# while True:
#     if(ser.inWaiting() > 0):
#         recieved = ser.readline()
#         print(recieved)
#         print(recieved.decode("utf-8"))
#         exit()
import struct

import serial
import time
ser = serial.Serial('COM3', 115200, timeout=0)
# var = input("Enter something: ")
# print(b'<')
# ser.write(str.encode(var))
# while 1:
#     # try:
#         # if ser.in_waiting > 3:
#         #     returnData = ser.read(4)
#         #     print(int.from_bytes(returnData, byteorder='little'))
#     if ser.in_waiting > 0:
#         returnData = ser.readline()
#         if returnData.decode()[0] == '<':
#             print("got it")
#         # print(returnData)
#         print(returnData.decode())
#         # time.sleep(0.1)
#             # print(returnData.decode("utf-8"))
#             # print(str(returnData))
#             # print(int.from_bytes(returnData, byteorder='little'))
#             #
#             # print(returnData[1:5])
#             # print(int.from_bytes(returnData[1:5], byteorder='little'))
#             #
#             # decoded_bytes = float(returnData[0:len(returnData) - 2].decode("utf-8"))
#             # print(decoded_bytes)
#
#             # for byte in returnData:
#             #     print(byte)
#             # print(returnData.decode())
#             # print("")
#             # print(str(returnData, 'utf-8', 'ignore'))
#     # except:
#     #     print('Data could not be read')

startMarker = 60
endMarker = 62

def recvFromArduino():
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
            ck = ck + x.decode("utf-8")  # change for Python3
            byteCount += 1
        x = ser.read()

    return (ck)

def main():

    time.sleep(2)

    ser.write("<1>".encode('utf-8'))

    while True:
        if ser.inWaiting() > 0:
            try:
                dataRecvd = recvFromArduino()
                print("Reply Received  " + dataRecvd)
            except:
                print("cannot read")

if __name__ == '__main__':
    main()





# ser = serial.Serial('COM3', 112500, timeout=0)
# print("opened serial")
# message = "0\r\n"
# print(message)
# time.sleep(2)
# ser.write(str.encode(message))
# keepRunning = True
# foundData = False
# while keepRunning:
#     try:
#         returnData = ser.readline()
#         if returnData != b'':
#             print(returnData)
#             # q.put(returnData)
#             foundData = True
#         else:
#             if foundData == True:
#                 keepRunning = False
#         time.sleep(0.1)
#     except ser.SerialTimeoutException:
#         print('Data could not be read')
# print("finishing comms")