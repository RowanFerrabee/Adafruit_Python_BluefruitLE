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


# Main function implements the program logic so it can run in a background
# thread.  Most platforms require the main thread to handle GUI events and other
# asyncronous events like BLE actions.  All of the threading logic is taken care
# of automatically though and you just need to provide a main function that uses
# the BLE provider.
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
    # try:
    print('Discovering services...')
    UART.discover(target_device)

    print('Service discovery complete')
    articulate_board = UART(target_device)

    time.sleep(1.0)

    print('Sending stream command until ok is received')

    stream_cmd =  'STREAMPLSSTREAMPLSSTREAMPLSSTREAMPLSSS'
    cmd_checksum = chr(sum([ord(x) for x in stream_cmd]) % 256)

    ok_response = 'OKOKOKOKOKOKOKOKOKOKOKOKOKOKOKOKOKOKOK'
    ok_checksum = chr(sum([ord(x) for x in ok_response]) % 256)

    NUM_TRIALS = 20
    HEADER_VAL = chr(253)
    PACKET_SIZE = 40
    OVERHEAD = 2
    DATA_SIZE = PACKET_SIZE - OVERHEAD
    MAX_NUM_ERRORS = 4

    trial = 0
    ok_success = False

    while (trial < NUM_TRIALS and ok_success == False):
        print('Trial', trial)

        # send bytes
        articulate_board.write(HEADER_VAL)
        articulate_board.write(stream_cmd)
        articulate_board.write(cmd_checksum)

        # wait for reception
        msg = ''
        num_errors = 0
        while((len(msg) < PACKET_SIZE) and (num_errors < MAX_NUM_ERRORS)):
            received = articulate_board.read(timeout_sec=1)
            if received is not None:
                msg = msg + (received)
            else:
                num_errors=num_errors+1

        expectedResponse = (HEADER_VAL + ok_response + ok_checksum)
        ok_success = len(msg) == PACKET_SIZE and msg == expectedResponse

    if (ok_success):
        print('Successfully entered data stream mode')
    else:
        target_device.disconnect()
        print('Error: could not enter stream mode')
        sys.exit()

    read_error = False
    while (not(read_error)):

        # wait for reception
        msg = ''
        num_errors = 0
        while((len(msg) < PACKET_SIZE) and (num_errors < MAX_NUM_ERRORS)):
            received = articulate_board.read(timeout_sec=1)
            if received is not None:
                msg = msg + (received)
            else:
                num_errors=num_errors+1

        print('Received:', msg)

        read_error = num_errors == MAX_NUM_ERRORS

    if (read_error):
        print('Error: failed to read a package')

    # except Exception, e:
    #     print('Failed with Exception: \'{}\''.format(e))

    # finally:
        # Make sure device is disconnected on exit.
    target_device.disconnect()

# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)
