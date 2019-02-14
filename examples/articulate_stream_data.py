# Search for BLE UART devices and list all that are found.
# Author: Tony DiCola
import atexit
import time
import socket

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

STARTING_STREAM = 0
STREAMING = 1
state = STARTING_STREAM

fsmState = 0
fsmCounter = 0

imuDataAvailable = 0
latestImuData = ''

def imuServerWorker():
    global imuDataAvailable
    global latestImuData

    time.sleep(1)
    HOST = ''               # Symbolic name meaning all available interfaces
    PORT = 5432             # Arbitrary non-privileged port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(2)
    conn, addr = s.accept()
    print('IMU server connected on: {}'.format(addr))

    try:
        while(True):
            if (imuDataAvailable):
                conn.send(latestImuData)
                imuDataAvailable = False
                print('Sent IMU Data')
    finally:
        if conn is not None:
            print('Closing socket connection')
            conn.close()
        else:
            print('No socket connection')


def bluetoothWorker(articulate_board):

    global imuDataAvailable
    global latestImuData

    MAX_NUM_ERRORS = 4

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

        if (len(msg) < PACKET_SIZE):
            print("Failed to read packet. Num errors: {}".format(num_errors))
            continue

        if (msg[POS_SOP] != chr(SOP)):
            print("Incorrect SOP")
            continue

        parsed_message = None
        if (msg[POS_DATA] == chr(IMU_DATA_MSG)):
            parsed_message = IMUDataMsg.fromBytes(msg)
            if (imuDataAvailable == False):
                latestImuData = msg[2:18]
                imuDataAvailable = True

        elif (msg[POS_DATA] == chr(ACK_MSG)):
            parsed_message = ACKMsg.fromBytes(msg)

        elif (msg[POS_DATA] == chr(STANDBY_MSG)):
            parsed_message = StandbyMsg.fromBytes(msg)

        else:
            print("Invalid Data Type")
            continue

        print("Received: {}".format(parsed_message))


def keepAliveWorker(articulate_board):
    # Send periodic messages to continue streaming 
    test_standby = True

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
            print("Sending stream message")
            msg = StreamMsg(50)    # Set IMU streaming period (in ms)
            articulate_board.write(msg.toBytes())

        time.sleep(1)
        i=i+1

def lraCmdWorker(articulate_board):

    onIntensities = [int(127)]*8
    onMsgBytes = LRACmdMsg(onIntensities).toBytes()

    offIntensities = [int(0)]*8
    offMsgBytes = LRACmdMsg(offIntensities).toBytes()

    i = 0
    while(True):
        if (i%2 == 0):
            print("Sending LRA on message")
            articulate_board.write(onMsgBytes)
            time.sleep(1)
        else:
            print("Sending LRA off message")
            articulate_board.write(offMsgBytes)
            time.sleep(3)

        i += 1

def main():

    target_device_name = u'RN4871-1444'
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
