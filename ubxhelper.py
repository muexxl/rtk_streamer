
#! /usr/bin/env python
import struct
from datetime import datetime
import time

UBX_PORT_USB_ONLY = b'\x00\x00\x00\x01\x00\x00'
UBX_PORT_NONE = b'\x00\x00\x00\x00\x00\x00'
UBX_PORT_CURRENT_PORT_DISABLE = b'\x00'
UBX_PORT_CURRENT_PORT_ENABLE = b'\x01'
UBX_HEADER = b'\xb5\x62'
UBX_MSG_MIN_LENGTH = 8

UBX_NO_FIX = 0x00
UBX_DR_ONLY_FIX = 0x01
UBX_2D_FIX = 0x02
UBX_3D_FIX = 0x03
UBX_3D_DR_FIX = 0x04
UBX_TIME_FIX = 0x05

RTCM_1005_UBX_MSG_ID = b'\xf5\x05'
RTCM_1077_UBX_MSG_ID = b'\xf5\x4d'
RTCM_1087_UBX_MSG_ID = b'\xf5\x57'
RTCM_1230_UBX_MSG_ID = b'\xf5\xe6'

UBX_MSG_IDS = {
    "ACK-ACK": b"\x05\x01",
    "ACK-NAK": b"\x05\x00",
    "AID-ALM": b"\x0B\x30",
    "AID-LPSRV": b"\x0B\x32",
    "AID-ALP": b"\x0B\x50",
    "AID-ADP": b"\x0B\x33",
    "AID-EPH": b"\x0B\x31",
    "AID-HUI": b"\x0B\x02",
    "AID-ANI": b"\x0B\x01",
    "AID-REG": b"\x0B\x00",
    "ESF-INS": b"\x10\x15",
    "ESF-MEAS": b"\x10\x02",
    "ESF-RAW": b"\x10\x03",
    "ESF-STATUS": b"\x10\x10",
    "HNR-INS": b"\x28\x02",
    "HNR-PVT": b"\x28\x00",
    "LOG-FINDTIME": b"\x21\x0E",
    "LOG-INFO": b"\x21\x08",
    "LOG-RETRIEVEPOS": b"\x21\x0F",
    "LOG-RETRIEVEPOS2": b"\x21\x0B",
    "LOG-RETRIEVEST": b"\x21\x0D",
    "MGA-ACK": b"\x13\x60",
    "MGA-DBD": b"\x13\x80",
    "MGA-FLASH": b"\x13\x21",
    "MON-COMMS": b"\x0A\x36",
    "MON-HW2": b"\x0A\x0B",
    "MON-HW3": b"\x0A\x37",
    "MON-Hwe": b"\x0A\x09",
    "MON-IO": b"\x0A\x02",
    "MON-MSGPP": b"\x0A\x06",
    "MON-RF": b"\x0A\x38",
    "MON-RXBUF": b"\x0A\x07",
    "MON-RXR": b"\x0A\x21",
    "MON-SMGR,": b"\x0A\x2E",
    "MON-SPAN": b"\x0A\x31",
    "MON-TXBUF": b"\x0A\x08",
    "NAV-AOPSTATUS": b"\x01\x60",
    "NAV-ATT": b"\x01\x05",
    "NAV-CLOCK": b"\x01\x22",
    "NAV-DGPS": b"\x10\x31",
    "Nav-DOP": b"\x01\x04",
    "NAV-EKFSTATUS": b"\x01\x40",
    "NAV-EDE": b"\x01\x61",
    "NAV-GEOFENCE": b"\x01\x39",
    "NAV-HPPOSECEF": b"\x01\x13",
    "NAV-HPPOSLLH": b"\x01\x14",
    "NAV-OPO": b"\x01\x09",
    "NAV-ORB": b"\x01\x34",
    "NAV-POSECEF": b"\x01\x01",
    "NAV-POSLLH": b"\x01\x02",
    "NAV-PVT": b"\x01\x07",
    "NaV-RELPOSNED": b"\x01\x3C",
    "NAV-SAT": b"\x01\x35",
    "NAV-BAS": b"\x01\x32",
    "NAV-SIG": b"\x01\x43",
    "NAV-SLAS": b"\x01\x42",
    "NAV-SOL": b"\x01\x06",
    "NAV-STATUS": b"\x01\x03",
    "NAV-SVIN": b"\x01\x3B",
    "NAV-TIMEBDS": b"\x01\x24",
    "NAV-TIMEGAL": b"\x01\x25",
    "NAV-TIMEGLO": b"\x01\x23",
    "NAV-TIMEGPS": b"\x01\x20",
    "NAV-TIMELS": b"\x01\x26",
    "NAV-TIMEGZSS": b"\x01\x27",
    "NAV-TIMEUTC": b"\x01\x21",
    "NAV-VELECEF": b"\x01\x11",
    "NAV-VELNED": b"\x01\x12",
    "RXM-ALM": b"\x02\x30",
    "RXM-EPH": b"\x02\x31",
    "RXM-IMES": b"\x02\x61",
    "RXM-MEAS": b"\x02\x14",
    "RXM-PMP": b"\x02\x72",
    "RXM-RAWX": b"\x02\x15",
    "RXM-RAW": b"\x02\x10",
    "RXM-RLM": b"\x02\x59",
    "RXM-RTCM": b"\x02\x32",
    "RXM-SFRBX": b"\x02\x13",
    "RXM-SERB": b"\x02\x11",
    "RXM-SVSI": b"\x02\x20",
    "SECUNIOID": b"\x27\x03",
    "TIM-DOSC": b"\x0D\x11",
    "TIM-FCH": b"\x0D\x16",
    "TIM-SMEAS": b"\x0D\x13",
    "TIM-SVIN": b"\x0D\x04",
    "TIM-TM2": b"\x0D\x03",
    "TIM-TOS": b"\x0D\x12",
    "TIM-TP": b"\x0D\x01",
    "TIM-VCOCAL": b"\x0D\x15",
    "TIM-VRFY": b"\x0D\x06",
    "UPD-SOS": b"\x09\x14",
    "NMEA-GxGGA": b"\xF0\x00",
    "NMEA-GxSLL": b"\xF0\x01",
    "NMEA-GxGSA": b"\xF0\x02",
    "NMEA-GxGSV": b"\xF0\x03",
    "NMEA-GaRMC": b"\xF0\x04",
    "NMEA-GavTB": b"\xF0\x05",
    "NMEA-GxGRS": b"\xF0\x06",
    "NMEA-GxGST": b"\xF0\x07",
    "NMEA-GxZDA": b"\xF0\x08",
    "NMEA-GxGBS": b"\xF0\x09",
    "NMEA-GxDTM": b"\xF0\x0A",
    "NMEA-GxGNS": b"\xF0\x0D",
    "NMEA-GvLW": b"\xF0\x0F",
    "PUBX00": b"\xF1\x00",
    "PUBX01": b"\xF1\x01",
    "PUBX03": b"\xF1\x03",
    "PUBX04": b"\xF1\x04",
    "RTCM3.3-1005": b"\xF5\x05",
    "RTCM3.3-1074": b"\xF5\x4A",
    "RTCM3.3-1077": b"\xF5\x4D",
    "RTCM3.3-1084": b"\xF5\x54",
    "RTCM3.3-1087": b"\xF5\x57",
    "RTCM3.3-1094": b"\xF5\x5E",
    "RTCM3.3-1097": b"\xF5\x61",
    "RTCM3.3-1124": b"\xF5\x7C",
    "RTCM3.3-1127": b"\xF5\x7F",
    "RTCM3.3-1230": b"\xF5\xE6",
    "RTCM3.3-4072.0": b"\xF5\xFE",
    "RTCM3.3-4072.1": b"\xF5\xFD"
}


def get_msg_by_id(id):
    for msg in UBX_MSG_IDS:
        if UBX_MSG_IDS[msg] == id:
            return msg
    return None


def get_id_by_msg(msg):
    try:
        id = UBX_MSG_IDS[msg]
    except KeyError:
        id = None
    return id


class UBXMSG(object):
    header = UBX_HEADER
    msg_type = "Generic"
    class_ID = b'\x00'
    msg_ID = b'\x00'
    length = 0  # 2 byte int, lil endian
    payload = b''  # message dependant
    checksum = b'\x00\x00'  # 2 byte checksum
    buffer = b''

    def __init__(self, message=b'', time_received= 0):
        if len(message) >= 8:
            self.buffer = message
            self.header = message[0:2]
            self.class_ID = message[2:3]
            self.msg_ID = message[3:4]
            self.length = struct.unpack('<H', message[4:6])[0]
            self.checksum = message[-2:]
            self.payload = message[6:-2]
            if time_received:
                self.time_received= time_received
            else:
                self.time_received = time.time()

    def calc_checksum(self, content: bytes) -> bytes:
        """
        Calculate checksum using 8-bit Fletcher's algorithm.

        :param bytes content: message content, excluding header and checksum bytes
        :return: checksum
        :rtype: bytes

        """

        check_a = 0
        check_b = 0

        for char in content:
            check_a += char
            check_a &= 0xFF
            check_b += check_a
            check_b &= 0xFF

        return bytes((check_a, check_b))

    def verify(self, content):
        self.buffer = content
        return self.is_checksum_valid()

    def update_checksum(self):
        self.checksum = self.calc_checksum(self.buffer[2:-2])

    def is_checksum_valid(self):
        checksum_from_buffer = self.buffer[-2:]
        calculated_checksum = self.calc_checksum(self.buffer[2:-2])
        return checksum_from_buffer == calculated_checksum

    def update_length(self):
        self.length = len(self.payload)

    def update(self):
        self.update_length()
        self.update_buffer()
        self.update_checksum()
        self.update_buffer()

    def update_buffer(self):
        msg = self.header
        msg += self.class_ID
        msg += self.msg_ID
        msg += self.length.to_bytes(2, 'little')
        msg += self.payload
        msg += self.checksum
        self.buffer = msg

    def serialize_poll(self):
        self.payload = b''
        self.update()
        return self.buffer

    def serialize(self):
        self.update()
        return self.buffer

    def print(self):
        self.update()

        for b in self.buffer:
            if b < 0x10:
                print("0", end='')
            print(f"{b:0X} ", end='')

    def specify(self):
        identifier = self.class_ID + self.msg_ID

        if (identifier == b'\x01\x02'):
            return UBX_NAV_POSLLH(self.buffer, self.time_received)
        if (identifier == b'\x01\x03'):
            return UBX_NAV_STATUS(self.buffer, self.time_received)
        if (identifier == b'\x01\x06'):
            return UBX_NAV_SOL(self.buffer, self.time_received)
        if (identifier == b'\x01\x14'):
            return UBX_NAV_HPPOSLLH(self.buffer, self.time_received)
        if (identifier == b'\x01\x21'):
            return UBX_NAV_TIMEUTC(self.buffer, self.time_received)
        if (identifier == b'\x01\x3B'):
            return UBX_NAV_SVIN(self.buffer, self.time_received)
        if (identifier == b'\x06\x01'):
            return UBX_CFG_MSG(self.buffer, self.time_received)
        if (identifier == b'\x06\x04'):
            return UBX_RST_MSG(self.buffer,self.time_received)
        if (identifier == b'\x06\x23'):
            return UBX_CFG_NAVX5(self.buffer,self.time_received)
        if (identifier == b'\x13\x60'):
            return UBX_MGA_ACK(self.buffer,self.time_received)
        if (identifier == b'\x13\x80'):
            return UBX_MGA_DBD(self.buffer,self.time_received)
        # print("Message unknown: ")
        # self.print()
        return self


class UBX_NAV_POSLLH(UBXMSG):
    class_ID = b'\x01'
    msg_ID = b'\x02'
    msg_type = 'NAV-POSLLH'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        self.itow = struct.unpack('<I', self.payload[0:4])[0]
        self.lon = struct.unpack('<i', self.payload[4:8])[0]
        self.lat = struct.unpack('<i', self.payload[8:12])[0]
        self.height = struct.unpack('<i', self.payload[12:16])[0]
        self.hMSL = struct.unpack('<i', self.payload[16:20])[0]
        self.vAcc = struct.unpack('<I', self.payload[20:24])[0]
        self.hAcc = struct.unpack('<I', self.payload[24:28])[0]


class UBX_NAV_STATUS(UBXMSG):
    class_ID = b'\x01'
    msg_ID = b'\x03'
    msg_type = 'NAV-STATUS'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        self.itow = struct.unpack('<I', self.payload[0:4])[0]
        self.gpsfix = self.payload[4]
        self.flags = self.payload[5]
        self.fixStat = self.payload[6]
        self.flags2 = self.payload[7]
        self.ttff = struct.unpack('<I', self.payload[8:12])[0]
        self.carrSoln = self.flags2 >>6
        self.RTK_float = (self.flags2 >>6 ) & 0x01
        self.RTK_fix = (self.flags2 >>7 ) & 0x01
        



class UBX_NAV_SOL(UBXMSG):
    class_ID = b'\x01'
    msg_ID = b'\x06'
    msg_type = 'NAV-SOL'
    
    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        self.itow = struct.unpack('<I', self.payload[0:4])[0]
        self.ftow = struct.unpack('<i', self.payload[4:8])[0]
        self.week = struct.unpack('<h', self.payload[8:10])[0]
        self.gps_fix_type = self.payload[10]
        self.numSV = self.payload[47]


class UBX_NAV_HPPOSLLH(UBXMSG):
    class_ID = b'\x01'
    msg_ID = b'\x14'
    msg_type = 'NAV-HPPOSLLH'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        self.invalid = self.payload[3]
        self.itow = struct.unpack('<I', self.payload[4:8])[0]
        lon_e7, lat_e7, height_e3, hMSL_e3, lon_e9, lat_e9, height_e4, hMSL_e4, hAcc_e4, vAcc_e4 = struct.unpack('<iiiibbbbII', self.payload[8:36])
        self.lon = lon_e7 * 1e-7 + lon_e9 * 1e-9
        self.lat = lat_e7 * 1e-7 + lat_e9 * 1e-9
        self.height= height_e3 * 1e-3 + height_e4*1e-4
        self.hMSL = hMSL_e3*1e-3  + hMSL_e4*1e-4
        self.hAcc= hAcc_e4 * 1e-4
        self.vAcc= vAcc_e4 * 1e-4

class UBX_NAV_TIMEUTC(UBXMSG):
    class_ID = b'\x01'
    msg_ID = b'\x21'
    msg_type = 'NAV-TIMEUTC'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        self.itow = struct.unpack('<I', self.payload[0:4])[0]
        self.tAcc = struct.unpack('<I', self.payload[4:8])[0]
        self.nano = struct.unpack('<i', self.payload[8:12])[0]
        self.year = struct.unpack('<S', self.payload[12:14])[0]
        self.month = struct.unpack('<B', self.payload[14:15])[0]
        self.day = struct.unpack('<B', self.payload[15:16])[0]
        self.hour = struct.unpack('<B', self.payload[16:17])[0]
        self.min = struct.unpack('<B', self.payload[17:18])[0]
        self.sec = struct.unpack('<B', self.payload[18:19])[0]
        self.validity_flags = struct.unpack('<B', self.payload[19:20])[0]


class UBX_NAV_SVIN(UBXMSG):
    class_ID = b'\x01'
    msg_ID = b'\x3B'
    msg_type = 'NAV-SVIN'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        self.itow = struct.unpack('<I', self.payload[4:8])[0]
        self.dur = struct.unpack('<I', self.payload[8:12])[0]
        self.mean_acc = struct.unpack('<I', self.payload[28:32])[0]  # in 0.1mm
        self.num_obs = struct.unpack('<I', self.payload[32:36])[
            0]  # Number of observations
        self.valid = self.payload[36]
        self.in_progress = self.payload[37]


class UBX_CFG_MSG(UBXMSG):
    class_ID = b'\x06'
    msg_ID = b'\x01'
    msg_type = 'CFG-MSG'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        if msg:
            if len(self.payload) == 8:
                self.target_msg_id = self.payload[0:2]
                self.port = self.payload[2:8]

            if len(self.payload) == 2:
                self.target_msg_id = self.payload[0:2]

    def encode(self, msg_id, port):
        """ msg id 2 bytes \n
            port 6 bytes / port 4 = USB \n
            OR
            port 1 byte =current port
        """
        self.payload = msg_id + port
        self.update()

    def on(self):
        port = b'\x00\x00\x00\x01\x00\x00'
        self.payload = self.target_msg_id + port
        self.update()
        return self

    def off(self):
        port = b'\x00\x00\x00\x00\x00\x00'
        self.payload = self.target_msg_id + port
        self.update()
        return self

    def poll(self):
        self.payload = self.target_msg_id
        self.update()
        return self


class UBX_RST_MSG(UBXMSG):
    class_ID = b'\x06'
    msg_ID = b'\x04'
    msg_type = 'RST-MSG'
    RESET_MODE_HARDWARE_RESET = b'\x00'
    RESET_MODE_CONTROLLED_SOFTWARE_RESET = b'\x01'
    RESET_MODE_CONTROLLED_RESET_GNSS_ONLY = b'\x02'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        if msg:
            self.navSBR = self.payload[0:2]
            self.reset_mode = self.payload[2:3]

    def encode(self, navSBR, reset_mode):
        self.payload = navSBR + reset_mode + b'\x00'
        self.update()


class UBX_RST_MSG_COLDSTART(UBX_RST_MSG):
    def __init__(self):
        super().__init__()
        navSBR = b'\xff\xb9'
        navSBR = b'\xff\xff'  # from u-center
        reset_mode = self.RESET_MODE_CONTROLLED_RESET_GNSS_ONLY
        self.encode(navSBR, reset_mode)


class UBX_RST_MSG_WARMSTART(UBX_RST_MSG):
    def __init__(self):
        super().__init__()
        navSBR = b'\x01\x00'  # from u-center
        reset_mode = self.RESET_MODE_CONTROLLED_RESET_GNSS_ONLY
        self.encode(navSBR, reset_mode)


class UBX_RST_MSG_HOTSTART(UBX_RST_MSG):
    def __init__(self):
        super().__init__()
        navSBR = b'\x00\x00'  # from u-center
        reset_mode = self.RESET_MODE_CONTROLLED_RESET_GNSS_ONLY
        self.encode(navSBR, reset_mode)


class UBX_MGA_DBD(UBXMSG):
    class_ID = b'\x13'
    msg_ID = b'\x80'
    msg_type = 'MGA-DBD'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)

    def decode(self):
        self.type = self.payload[0]
        self.version = self.payload[1]
        self.infoCode = self.payload[2]
        self.msgId = self.payload[3]
        self.msgPayloadStart = self.payload[4:]


class UBX_CFG_TMODE3(UBXMSG):
    class_ID = b'\x06'
    msg_ID = b'\x71'
    msg_type = 'CFG-TMODE3'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)

    def encode_survey_in(self, svin_min_dur, svin_acc_limit):
        """
        svin_min_dur = minimum duration in seconds \n
        svin_acc_limit = required accouracy in meter 
        """

        svin_acc_limit_e4 = int(round(svin_acc_limit*1e4,0))
        self.payload = bytearray(40)
        self.payload[2] = 1  # 0-disable 1-survey-in
        self.payload[24:28] = svin_min_dur.to_bytes(
            4, 'little')  # min duration in seconds
        self.payload[28:32] = svin_acc_limit_e4.to_bytes(
            4, 'little')  # required accuraccy in 0.1 mm (1e-4 m)
        self.payload = bytes(self.payload)
        self.update()

    def encode_fixed(self, lat, lon, alt, acc):
        """
        mode 0=disable 1=survey-in \n
        lat/lon in deg (float) \n
        alt in m (float) \n
        acc - accuracy in m (float) \n

        """
        self.payload = bytearray(40)
        self.payload[2] = 2  # 0-disable 1-survey-in
        self.payload[3] = 1
        lat_e7 = int(round(lat*1e7, 0))
        lon_e7 = int(round(lon*1e7, 0))
        alt_e2 = round(alt*100, 0)

        self.payload[4:16] = struct.pack('<iii', lat_e7, lon_e7, alt_e2)

        lat_e9 = int(round(lat*1e9, 0))
        lat_hp = lat_e9-lat_e7*100
        lon_e9 = int(round(lon*1e9, 0))
        lon_hp = lon_e9-lon_e7*100
        alt_e4 = round(alt*1e4, 0)
        alt_hp = alt_e4-alt_e2
        self.payload[16:19] = struct.pack('<bbb', lat_hp, lon_hp, alt_hp)
        acc_e4 = round(acc*1e4, 0)
        self.payload[20] = struct.pack('<I', acc_e4)

        self.payload = bytes(self.payload)
        self.update()


class UBX_CFG_RATE(UBXMSG):
    class_ID = b'\x06'
    msg_ID = b'\x08'
    msg_type = 'CFG-RATE'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)

    def encode(self, measurement_rate, nav_rate=1, time_ref=1):
        self.payload = bytearray(6)
        self.payload[0:2] = measurement_rate.to_bytes(
            2, 'little')  # measurement rate im milliseconds
        # navigation rate in cycles, default =1
        self.payload[2:4] = nav_rate.to_bytes(2, 'little')
        self.payload[4:6] = time_ref.to_bytes(
            2, 'little')  # time reference 1-GPS 0-UTC
        self.payload = bytes(self.payload)
        self.update()


class UBX_CFG_NAVX5(UBXMSG):
    class_ID = b'\x06'
    msg_ID = b'\x23'
    msg_type = 'CFG-NAVX5'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)

    def set_ack_aid(self, is_ack_aid):
        if (len(self.payload) != 40):
            return  # payload not valid
        pl = bytearray(self.payload)
        pl[2] = 0  # bitfield 0..7
        pl[3] = (1 << 2)  # bitfield 8..15
        pl[17] = is_ack_aid

        self.payload = bytes(pl)

    def enable_mga_ack(self):
        pl = bytearray(40)
        pl[2] = 0x4c
        pl[3] = 0x66
        pl[4] = 0xc0
        pl[10] = 0x03
        pl[11] = 0x10
        pl[17] = 0x01
        pl[32] = 0x64
        self.payload = bytes(pl)
        self.update()

    def disable_mga_ack(self):
        pl = bytearray(40)
        pl[2] = 0x4c
        pl[3] = 0x66
        pl[4] = 0xc0
        pl[10] = 0x03
        pl[11] = 0x10
        pl[17] = 0x00
        pl[32] = 0x64
        self.payload = bytes(pl)
        self.update()


class UBX_MGA_ACK(UBXMSG):
    class_ID = b'\x13'
    msg_ID = b'\x60'
    msg_type = 'MGA-ACK'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)
        self.decode()

    def decode(self):
        self.type = self.payload[0]  # 0x01 means the message has been used!
        self.version = self.payload[1]
        self.infoCode = self.payload[2]  # 0 means accepted
        self.msgID = self.payload[3]
        self.msgPayloadStart = self.payload[4:8]

    def refers_to_data_msg(self, msg: UBXMSG):
        return msg.payload[:4] == self.msgPayloadStart


class UBX_MGA_INI_TIME_UTC(UBXMSG):
    class_ID = b'\x13'
    msg_ID = b'\x40'
    msg_type = 'MGA-INI-TIME_UTC'

    def __init__(self, msg=b'', t=0):
        super().__init__(msg,t)

    def encode(self, latency, t_accuracy=0.5):

        dt = datetime.utcfromtimestamp(time.time()+latency)
        year = dt.year
        month = dt.month
        day = dt.day
        hour = dt.hour
        minute = dt.minute
        second = dt.second
        ns = dt.microsecond*1000

        self.payload = bytearray(24)
        self.payload[0] = 0x10  # type indicating ..TIME UTC
        self.payload[1] = 0  # version
        self.payload[2] = 0  # ref // no input via extint
        self.payload[3] = 18  # leap seconds
        self.payload[4:6] = year.to_bytes(2, 'little')
        self.payload[6] = month
        self.payload[7] = day
        self.payload[8] = hour
        self.payload[9] = minute
        self.payload[10] = second
        self.payload[11] = 0  # reserved
        self.payload[12:16] = ns.to_bytes(4, 'little')
        self.payload[16:18] = int(t_accuracy).to_bytes(
            2, 'little')  # time accuracy seconds
        self.payload[18] = 0
        self.payload[19] = 0
        # time accuracy nano seconds
        self.payload[20:24] = int((t_accuracy*1e9) % 1e9).to_bytes(4, 'little')

        self.payload = bytes(self.payload)
        self.update()


def starts_with_UBX_Header(buffer):
    if buffer[:2] == UBX_HEADER:
        return True


def starts_with_UBX_Message(buffer):

    if len(buffer) < UBX_MSG_MIN_LENGTH:
        return 0

    payload_length = struct.unpack('<H', buffer[4:6])[0]
    total_length = payload_length + 8
    if len(buffer) < total_length:
        return 0

    if UBXMSG().verify(buffer[:total_length]):
        return total_length
    else:
        return 0
