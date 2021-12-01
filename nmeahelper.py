#! /usr/bin/env python
import struct
NMEA_HEADER = b'$G' #aka b'\x24\x47'

def starts_with_NMEA_Header( buffer):
    if len(buffer)<2:
        return 0
    return buffer[0:2] == NMEA_HEADER

def starts_with_NMEA_Message( buffer):
    
    if not starts_with_NMEA_Header(buffer):
        return 0

    if len(buffer) < 7:
        return 0
    
    return find_NMEA_MSG_end(buffer)

def find_NMEA_MSG_end(buffer):
    for i in range(len(buffer)-1):
        if buffer[i:i+2] == b'\r\n':
            return i+2
    
    return 0