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

    conn = None

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        get_info = False
        ping_articulate = False
        read_from_articulate = True

        if (get_info):
            # Wait for service discovery to complete for the DIS service.  Will
            # time out after 60 seconds (specify timeout_sec parameter to override).
            print('Discovering services...')
            DeviceInformation.discover(target_device)

            # Once service discovery is complete create an instance of the service
            # and start interacting with it.
            dis = DeviceInformation(target_device)

            # Print out the DIS characteristics.
            print('Manufacturer: {0}'.format(dis.manufacturer))
            print('Model: {0}'.format(dis.model))
            print('Serial: {0}'.format(dis.serial))
            print('Hardware Revision: {0}'.format(dis.hw_revision))
            print('Software Revision: {0}'.format(dis.sw_revision))
            print('Firmware Revision: {0}'.format(dis.fw_revision))
            print('System ID: {0}'.format(dis.system_id))
            print('Regulatory Cert: {0}'.format(dis.regulatory_cert))
            print('PnP ID: {0}'.format(dis.pnp_id))
            for i, service in enumerate(target_device.list_services()):
                print('Device Service {0}: UUID {1}'.format(i, service.uuid))

        if (ping_articulate):
            # Wait for service discovery to complete for the UART service.  Will
            # time out after 60 seconds (specify timeout_sec parameter to override).
            print('Discovering services...')
            UART.discover(target_device)

            print('Service discovery complete')
            # Once service discovery is complete create an instance of the service
            # and start interacting with it.
            articulate_board = UART(target_device)

            time.sleep(1.0)

            print("Sending 'hello' to the device.")
            # Write a string to the TX characteristic.
            articulate_board.write('hello from Macbook!')
            # print("Sent 'Hello world!' to the device.")

            # print('Waiting up to 15 seconds to receive data from the device...')
            received = articulate_board.read(timeout_sec=15)
            if received is not None:
                # Received data, print it out.
                print('Received: {0}'.format(type(received)))
            else:
                # Timeout waiting for data, None is returned.
                print('Received no data!')

        if (read_from_articulate):

            # Establishing socket connection
            HOST = ''                 # Symbolic name meaning all available interfaces
            PORT = 5204              # Arbitrary non-privileged port
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen(2)
            conn, addr = s.accept()
            print('Connected by', addr)

            print('Discovering services...')
            UART.discover(target_device)

            print('Service discovery complete')
            articulate_board = UART(target_device)

            time.sleep(1.0)

            print('Sending BT data to BSD Socket')

            received = 1
            data = ""

            while(received != None):
                received = articulate_board.read(timeout_sec=10)
                if received is not None:
                    # Gather BT data
                    data = data + received
                    # Send data in sets of 16 bytes to socket
                    if (len(data) > 16):
                        conn.send(data[:16])
                        print(data[:16])
                        data = ""
                else:
                    # Timeout waiting for data, None is returned.
                    print('Received no data in 10s!')

    except Exception, e:
        print('Failed with Exception: \'{}\''.format(e))
    finally:
        # Make sure device is disconnected on exit.
        target_device.disconnect()
        if conn is not None:
            print('Closing socket connection')
            conn.close()

# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)
