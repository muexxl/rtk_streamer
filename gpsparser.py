#! /usr/bin/env python3


import struct

from serial.serialutil import SerialException
import ubxhelper
import serial
import time
import serial.tools.list_ports

import logging
logger = logging.getLogger(__name__)
logger.level=logging.DEBUG

from nmeahelper import *
from rtcmhelper import *
from ubxhelper import *
import threading
import logging
from os import fchown
import socket
import threading
import time
import subprocess


class GPSParser(threading.Thread):
    def __init__(self):
        logger.debug(f' GPSParser | initializing object')
        self.buffer = b''
        self.tx_buffer = b''
        self.rx_buffer = b''
        self.rtcm_buffer = b''
        self.tx_lock = threading.Lock()
        self.rx_lock = threading.Lock()
        self.rtcm_lock= threading.Lock()
        self.keep_running = True
        self.stream = serial.Serial()
        self.ubx_buffer=[]
        self.ubx_lock= threading.Lock()
        self.ready=False
        self.udp_stream_active = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1),
        self.sock.setblocking(0)
        self.udp_broadcasts = []
        shell_command = "ifconfig |grep broadcast|awk '{print $6}'"
        ip_list = subprocess.check_output(shell_command, shell=True, text=True).split('\n')
        for ip in ip_list:
            if ip=='':
                continue
            self.udp_broadcasts.append((ip,10777)) 
        threading.Thread.__init__(self)
    
    def open_stream_to_gps_device(self):
        gps_found = False

        print ("GPSParser | Scanning for GPS device on USB Ports")

        while not gps_found:
            for port in serial.tools.list_ports.comports():
                if (port.vid == 0x1546) & (port.pid == 0x01a8):
                    self.port ='/dev/' + port.name
                    try:
                        self.stream = serial.Serial(self.port, 115200)
                    except SerialException:
                        continue
                    print (f"GPSParser | Connection established to Ublox GPS device on port {port.name}")
                    gps_found = True
                    self.ready=True
            
            time.sleep(0.1)
        
    def run(self):

        logger.debug(f' GPSParser | run function started')
        while (self.keep_running):
            if not self.stream.isOpen():
                self.ready=False
                print (f"GPSParser | No Connection to GPS device")                   
                self.open_stream_to_gps_device()
            self.send_rx_buffer_to_stream()
            self.fill_buffer_from_stream()
            msg = self.extract_next_msg()
            if (starts_with_UBX_Header(msg)):
                self.ubx_lock.acquire()
                self.ubx_buffer.append(UBXMSG(msg))
                self.ubx_lock.release()

            elif (starts_with_RTCM_Header(msg) and self.udp_stream_active):
                self.publish_via_udp(msg)
            #if(msg):
            #    print(f"GPS Parser | {msg}")
            #if(starts_with_NMEA_Header(msg)):
            else:
                pass
                time.sleep(0.01)
        
        logger.debug(f' GPSParser | run function ended ')

    def stop(self):
        print (f' GPSParser | stop function started')
        self.keep_running = False
        self.join()
        print(f' GPSParser | stop function ended')
    
    def publish_via_udp(self, data):
        for broadcast in self.udp_broadcasts:
            try:
                self.sock.sendto(data, broadcast)
            except OSError:
                pass
    
    def get_next_ubx_msg(self):
        msg= None
        self.ubx_lock.acquire()
        try:
            msg=self.ubx_buffer.pop(0)
        except IndexError:
            pass
        self.ubx_lock.release()
        return msg

    def get_next_ubx_msg_type(self,msg_type):
        msg = None
        while 1:
            msg=self.get_next_ubx_msg()
            if msg:
                msg = msg.specify()
                if msg.msg_type ==  msg_type:
                    return msg
            else:
                time.sleep(0.5)
               
    def get_next_ubx_msg_type_timed(self,msg_type, timeout=-1):
        """
        msg_type e.g. NAV-SVIN
        timeout in seconds
        """
        msg = None
        return_msg=None
        is_expired=False
        start = time.time()
        while not is_expired:
            msg=self.get_next_ubx_msg()
            if msg:
                msg = msg.specify()
                if msg.msg_type ==  msg_type:
                    return_msg = msg
                    break
            else:
                time.sleep(0.1)
            
            if timeout==-1:#never expire
                pass
            else:
                if (start+timeout) < time.time():
                    is_expired=True
                    
        return return_msg

    def send_to_gps(self, data):
        self.rx_lock.acquire()
        self.rx_buffer += data
        self.rx_lock.release()
        
    def send_rx_buffer_to_stream(self):       
        self.rx_lock.acquire()
        try: 
            self.stream.write(self.rx_buffer[:])
        except serial.SerialException:
            logger.warning(f'GPS Parser | Write Error')
            self.stream.close()
        self.rx_buffer = b''
        self.rx_lock.release()
    
    def fill_buffer_from_stream(self):
        try:
            self.buffer += self.stream.read_all()
        except OSError:
            logger.warning(f'GPS Parser |  Read Error')
            self.stream.close()

    def request_mga_db(self):
        msg = ubxhelper.UBX_MGA_DBD().serialize()
        self.fill_buffer_from_stream()
        self.stream.write(msg)

    def clear_buffer_until_next_msg_and_return_length(self):

        buffer = self.buffer[:]
        length = 0
        while len(buffer)>8:
            length = self.starts_with_message(buffer)
            if length:
                self.buffer = buffer
                break
            buffer = buffer[1:]
        return length

            # option1 : message not complete
            # option2: message corrupted

    def starts_with_message(self, data):

        length = starts_with_NMEA_Message(data)
        if length:
            return length

        length = starts_with_RTCM_Message(data)
        if length:
            return length

        length = starts_with_UBX_Message(data)
        if length:
            return length

        return 0

    def starts_with_header(self, data):
        result = False

        if starts_with_NMEA_Header(data):
            result = True

        if starts_with_RTCM_Header(data):
            result = True

        if starts_with_UBX_Header(data):
            result = True

        return result

    def find_next_header(self, buffer):
        result = 0
        for i in range(len(buffer)-2):
            if self.starts_with_header(buffer[i:]):
                result = i
                break

        return result

    def extract_next_msg(self):
        length = self.clear_buffer_until_next_msg_and_return_length()
        next_msg = b''
        if length:
            next_msg = self.buffer[:length]
            self.buffer = self.buffer[length:]
        return next_msg
    
    def __del__(self):
        pass

        



logger = logging.getLogger(__name__)

#from pymavlink.dialects.v20 import ardupilotmega as mavlink2

class UDPServer(threading.Thread):
    def __init__(self):
 
        self.rx_buf = bytearray()
        self.rx_lock = threading.Lock()
        self.data_available = False

        self.tx_buf = bytearray()
        self.tx_lock = threading.Lock()
        self.keep_running = False
        self.max_buf_size = 0xffff
        threading.Thread.__init__(self)

    def __del__(self):
        pass
    def run(self):
        self.keep_running = True
        logger.info('Running UDP Server')
        while self.keep_running:
            if self.tx_buf:
                self._send()
            self._receive()

            time.sleep(0.2)

        logger.info('UDP SERVER Run function ended')


    def stop(self):
        logger.info('Stopping UDP Server')
        self.keep_running = False
        self.join()
        logger.info('Stopped UDP Server')


    def _receive(self):

        data = None
        try:
            data, address = self.sock.recvfrom(4096)
            #logger.warning(f'UDPServer  |  Received {data }from {address}')
        except BlockingIOError:
            pass
            # logger.info('Excepted BlockingIOError')
        if data:
            self.rx_lock.acquire()
            self.rx_buf +=bytearray(data)
            self.data_available = True
            self.rx_lock.release()

    def _send(self):
        self.tx_lock.acquire()
        
        for broadcast in self.udp_broadcasts:
            try:
                self.sock.sendto(self.tx_buf[:self.max_buf_size], broadcast)
            except OSError:
                pass
        self.tx_buf = self.tx_buf[self.max_buf_size:]
        self.tx_lock.release()

    def publish_via_udp(self, data):
        self.tx_lock.acquire()
        self.tx_buf +=bytearray(data)
        self.tx_lock.release()




