# Search for BLE UART devices and list all that are found.
# Author: Tony DiCola
import atexit
import time
import socket
import math

import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
from Adafruit_BluefruitLE.services import DeviceInformation

# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()

import random
import string
import threading
from msg_defs import *

# Main function implements the program logic so it can run in a background
# thread.  Most platforms require the main thread to handle GUI events and other
# asyncronous events like BLE actions.  All of the threading logic is taken care
# of automatically though and you just need to provide a main function that uses
# the BLE provider.

LRA_WORKER_PERIOD = 0.1
MIN_REC_MOVEMENT = 5
VIBRATE_THRESHOLD_DIST = 0.35 # Aprrox 20 deg in rad

STARTING_STREAM = 0
STREAMING = 1
state = STARTING_STREAM

fsmState = 0
fsmCounter = 0

exercising = False
recording = False
recordedMovement = []

imuDataAvailable = 0
latestImuData = None

def quaternionToGravity(quat):
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]
    
    gx = 2 * (x*z - w*y);
    gy = 2 * (w*x + y*z);
    gz = w*w - x*x - y*y + z*z;

    return [gx, gy, gz]

def imuServerWorker():
    global imuDataAvailable
    global latestImuData
    global exercising
    global recording
    global recordedMovement

    time.sleep(1)
    HOST = ''               # Symbolic name meaning all available interfaces
    PORT = 5432             # Arbitrary non-privileged port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(2)
    conn, addr = s.accept()
    conn.setblocking(0)
    print('IMU server connected on: {}'.format(addr))

    try:
        connectionFailed = False
        sendCounter = 0

        while (not(connectionFailed)):
            if (imuDataAvailable):
                # latestImuQuat = IMUDataMsg.fromBytes(latestImuData)
                # gravity = quaternionToGravity(latestImuQuat.quat)
                # print('Gravity: {}'.format(gravity))
                conn.send(latestImuData)
                imuDataAvailable = False

                sendCounter += 1
                if sendCounter >= 25:
                    print('Sent {} IMU Packets'.format(sendCounter))
                    sendCounter = 0

            # Check if any commands are being sent
            try:
                cmd = conn.recv(PACKET_SIZE)

                if (ord(cmd[POS_SOP]) == SOP and ord(cmd[POS_DATA]) == GUI_CONTROL_MSG):

                    if (cmd[POS_DATA+1] == chr(START_RECORDING)):
                        if not(recording):
                            recording = True
                            recordedMovement = []
                            print("Starting Recording!")

                    elif (cmd[POS_DATA+1] == chr(STOP_RECORDING)):
                        if (recording):
                            recording = False
                            print("Ending Recording!")
                            print("Recording size: {}".format(len(recordedMovement)))

                    elif (cmd[POS_DATA+1] == chr(START_EXERCISE)):
                        if not(exercising):
                            exercising = True
                            print("Starting Exercise!")

                    elif (cmd[POS_DATA+1] == chr(STOP_EXERCISE)):
                        if (exercising):
                            exercising = False
                            print("Ending Exercise!")

                    elif (cmd[POS_DATA+1] == chr(PRINT_RECORDING)):
                        print(recordedMovement)

                    else:
                        print("Received invalid command: {}".format(cmd))
                        connectionFailed = True
            except Exception:
                pass
    finally:
        if conn is not None:
            print('Closing socket connection')
            conn.close()
        else:
            print('No socket connection')


def bluetoothWorker(articulate_board):

    global imuDataAvailable
    global latestImuData
    global recording
    global recordedMovement

    MAX_NUM_ERRORS = 4
    recvCounter = 0

    while(True):

        msg = ''

        num_errors = 0
        while((len(msg) < PACKET_SIZE) and (num_errors < MAX_NUM_ERRORS)):
            received = articulate_board.read(timeout_sec=1)
            if received is not None:
                msg = msg + (received)
            else:
                time.sleep(0.01)
                num_errors += 1

        msg = msg[:PACKET_SIZE]

        if (len(msg) < PACKET_SIZE):
            print("Failed to read packet. Num errors: {}".format(num_errors))
            continue

        if (msg[POS_SOP] != chr(SOP)):
            print("Incorrect SOP")
            continue

        # if ((sum([ord(x) for x in msg]) % 256) != msg[POS_CHECKSUM]):
        #     print("Incorrect CHECKSUM")
        #     continue

        parsed_message = None
        if (msg[POS_DATA] == chr(IMU_DATA_MSG)):
            parsed_message = IMUDataMsg.fromBytes(msg)
            if (recording):
                recordedMovement.append(parsed_message)

            if (imuDataAvailable == False):
                latestImuData = msg
                imuDataAvailable = True

            recvCounter += 1

        elif (msg[POS_DATA] == chr(ACK_MSG)):
            parsed_message = ACKMsg.fromBytes(msg)
            recvCounter += 1

        elif (msg[POS_DATA] == chr(STANDBY_MSG)):
            parsed_message = StandbyMsg.fromBytes(msg)
            recvCounter += 1

        else:
            print("Invalid Data Type")
            continue

        if recvCounter >= 25:
            print("Received {} messages".format(recvCounter))
            recvCounter = 0


def keepAliveWorker(articulate_board):
    # Send periodic messages to continue streaming 
    test_standby = False

    i = 0
    while(True):
        if (test_standby):
            if (i%120 < 6):
                print("Sending standby message")
                articulate_board.write(StandbyMsg.toBytes())
            else:
                print("Sending stream message")
                msg = StreamMsg(100)    # Set IMU streaming period (in ms)
                articulate_board.write(msg.toBytes())

        else:
            # print("Sending stream message")
            msg = StreamMsg(50)    # Set IMU streaming period (in ms)
            articulate_board.write(msg.toBytes())

        time.sleep(1)
        i=i+1

def lraCmdWorker(articulate_board):

    lrasAreOn = False

    onIntensities = [int(127)]*8
    onMsgBytes = LRACmdMsg(onIntensities).toBytes()

    offIntensities = [int(0)]*8
    offMsgBytes = LRACmdMsg(offIntensities).toBytes()

    i = 0
    while(True):
        if (exercising and not(recording) and latestImuData != None
            and len(recordedMovement) >= MIN_REC_MOVEMENT):

            q1 = (IMUDataMsg.fromBytes(latestImuData)).quat

            minDist = VIBRATE_THRESHOLD_DIST + 1
            for movementPoint in recordedMovement:
                q2 = movementPoint.quat
                dotProduct = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]

                # Sometimes dotProduct is very slightly over 1
                if (dotProduct > 1 and dotProduct < 1.001):
                    dotProduct = 1

                pointDist = 2*math.acos(dotProduct)
                if (pointDist < minDist):
                    minDist = pointDist

            if (minDist >= VIBRATE_THRESHOLD_DIST):
                if not(lrasAreOn):
                    print("Turning LRAs on")
                    articulate_board.write(onMsgBytes)
                    lrasAreOn = True
            else:
                if (lrasAreOn):
                    print("Turning LRAs off")
                    articulate_board.write(offMsgBytes)
                    lrasAreOn = False

        else:
            if (lrasAreOn):
                print("Turning LRAs off")
                articulate_board.write(offMsgBytes)
                lrasAreOn = False

        time.sleep(LRA_WORKER_PERIOD)

        i += 1

def main():

    target_device_name = u'RN4871-1444'
    # target_device_name = u'RN4678-09E5'
    target_device = None

    # Clear any cached data because both bluez and CoreBluetooth have issues with
    # caching data and it going stale.
    ble.clear_cached_data()

    # Get the first available BLE network adapter and make sure it's powered on.
    adapter = ble.get_default_adapter()
    adapter.power_on()
    print('Using adapter: {0}'.format(adapter.name))

    # Start scanning with the bluetooth adapter.
    adapter.start_scan()
    # Use atexit.register to call the adapter stop_scan function before quiting.
    # This is good practice for calling cleanup code in this main function as
    # a try/finally block might not be called since this is a background thread.
    atexit.register(adapter.stop_scan)

    print('Searching for devices...')
    print('Press Ctrl-C to quit (will take ~30 seconds on OSX).')

    # Enter a loop and print out whenever a new device is found, and break when target is found.
    known_uarts = set()
    while type(target_device) == type(None):

        # Call UART.find_devices to get a list of any UART devices that
        # have been found.  This call will quickly return results and does
        # not wait for devices to appear.
        found = set(DeviceInformation.find_devices())

        # Check for new devices that haven't been seen yet and print out
        # their name and ID (MAC address on Linux, GUID on OSX).
        new = found - known_uarts
        for device in new:
            if (device.name != None and device.id != None):
                dev_name = unicode(device.name).encode('ascii', 'xmlcharrefreplace')
                dev_id = unicode(device.id).encode('ascii', 'xmlcharrefreplace')
                print('Found Device: {0} [{1}]'.format(dev_name, dev_id))
                if (dev_name == target_device_name):
                    target_device = device
                    print('Found Target Device!')
        known_uarts.update(new)

        if (type(target_device) != type(None)):
            break

        # Sleep for a half second and see if new devices have appeared.
        time.sleep(0.2)

    print('Connecting to device...')
    target_device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                             # to change the timeout

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    print('Discovering services...')
    UART.discover(target_device)

    print('Service discovery complete')
    articulate_board = UART(target_device)

    time.sleep(1.0)

    try:
        print("Starting bt thread")
        btThread = threading.Thread(target=bluetoothWorker, args=(articulate_board,))
        btThread.start()

        print('Starting keep alive thread')
        keepAliveThread = threading.Thread(target=keepAliveWorker, args=(articulate_board,))
        keepAliveThread.start()

        print('Starting LRA command thread')
        lraCmdThread = threading.Thread(target=lraCmdWorker, args=(articulate_board,))
        lraCmdThread.start()

        while(True):
            print("Starting IMU server thread")
            imuServerThread = threading.Thread(target=imuServerWorker, args=())
            imuServerThread.start()
            imuServerThread.join()

    finally:
        target_device.disconnect()


# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)
