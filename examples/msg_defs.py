# packet related defs

PACKET_SIZE = 40
PACKET_OVERHEAD = 2
SOP = 253
POS_SOP = 0
POS_DATA = 1
POS_CHECKSUM = (PACKET_SIZE-1)

# message related defines

MSG_OVERHEAD = 1

NONE_MSG = 0
ACK_MSG = 1
STANDBY_MSG = 2
STATE_CHANGE_MSG = 3
IMU_DATA_MSG = 4
LRA_CONTROL_MSG = 5
GUI_CONTROL_MSG = 6
NUM_MSG_TYPES = 7

# streaming states

DEFAULT_STATE = 0
IMU_STREAMING_STATE = 1
INVALID_STATE = 2
NUM_BOARD_STATES = 3

START_RECORDING = 0
STOP_RECORDING = 1
START_EXERCISE = 2
STOP_EXERCISE = 3
PRINT_RECORDING = 4

import struct
import binascii

AOK = 'AOK'
ERR = 'ERR'

def wrapDataInPacket(data):
    padLen = PACKET_SIZE - PACKET_OVERHEAD - len(data)

    if(padLen < 0):
        return None

    checksum = (sum([ord(x) for x in data]) % 256)

    if(padLen > 0):
        padding = ''.join(['\0' for x in range(padLen)])
        data = data + padding

    msg = struct.pack('B', SOP) + data + struct.pack('B', checksum)
    return str(msg)

class StreamMsg:
    def __init__(self, p, ok=True):
        self.period = int(p)
        self.isOk = ok

    def toBytes(self):
        data = struct.pack('B', STATE_CHANGE_MSG) + struct.pack('<B', IMU_STREAMING_STATE) + struct.pack('<i', self.period)
        return wrapDataInPacket(data)

    def fromBytes(cls, bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(STREAM_MSG)):
            return None

        period = struct.unpack('<i', bytes[1:5].encode())
        isOk = bytes[5:7] == AOK

        return StreamMsg(period, isOk)

class LRACmdMsg:
    def __init__(self, intensities):
        # TODO: Check values are integers <127
        self.intensities = intensities

    def toBytes(self):
        data = struct.pack('B', LRA_CONTROL_MSG)
        for i in range(8):
            data += struct.pack('B', int(self.intensities[i]))
        return wrapDataInPacket(data)

    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(LRA_CONTROL_MSG)):
            return None

        new_intensities = []
        for i in range(4):
            val = struct.unpack('B', bytes[1+i:2+i])
            new_intensities.append(val[0])

        return LRACmdMsg(new_intensities)

class ACKMsg:
    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(ACK_MSG)):
            return None

        return ACKMsg()

    def __str__(self):
        return "ACK"

class StandbyMsg:
    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(ACK_MSG)):
            return None

        return StandbyMsg()

    @staticmethod
    def toBytes():
        data = struct.pack('B', STANDBY_MSG)
        return wrapDataInPacket(data)

    def __str__(self):
        return "STANDBY"

class IMUDataMsg:
    def __init__(self, quat):
        self.quat = quat

    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(IMU_DATA_MSG)):
            return None

        quat = []
        for i in range(4):
            val = struct.unpack('f', bytes[1+4*i:5+4*i])
            quat.append(val[0])

        return IMUDataMsg(quat)

    def __str__(self):
        return "IMU Data: {}".format(str(self.quat))
