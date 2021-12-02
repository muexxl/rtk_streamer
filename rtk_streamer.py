#! /usr/bin/env python3

from gpsparser import GPSParser
import serial
import sys
import logging
import time
from ubxhelper import *
import threading
import os

logging.basicConfig(format='[%(levelname)8s]\t%(asctime)s: %(message)s ', datefmt='%d.%m.%Y %H:%M:%S', filename='rtkstreamer.log', filemode='a', level=logging.INFO)
logger = logging.getLogger(__name__)

class RTKStreamer():
    """RTK Streamer controls ublox GPS device via GPS Parser"""
    def __init__(self, gpsparser : GPSParser):
        self.gpsp = gpsparser
        self.status = 'undefined'
        self.rate=0
        self.msg_mode=''
    
    def run(self):
        self.gpsp.start()
        while(1):
            self.wait_for_gps_ready()
            if self.status == 'undefined':
                self.reset_gps('hot')
                self.msg_mode=''
                self.rate=0
                time.sleep(1)
                self.set_messages('svin')
                self.start_SVIN()
                self.set_rate(500)
                self.gpsp.udp_stream_active = False
            elif self.status == 'surveying':
                self.gpsp.udp_stream_active = False
            elif self.status == 'streaming':
                self.set_rate(1000)
                self.set_messages('streaming')
                self.gpsp.udp_stream_active = True

            self.status=self.get_status()
    
    def wait_for_gps_ready(self):
        while not self.gpsp.ready:
            time.sleep(0.2)
            
    def get_status(self):
        self.gpsp.ubx_buffer.clear()
        msg=self.gpsp.get_next_ubx_msg_type_timed('NAV-SVIN',5)
        status='undefined'
        if msg:
            logger.info(f"RTK Streamer | SVIN Status Dur: {msg.dur}s, Acc: {msg.mean_acc/10000:01.3f}m  Valid: {msg.valid}  Obs: {msg.num_obs}  In progress: {msg.in_progress}  itow: {msg.itow} ")
            if msg.in_progress==1:
                status='surveying'
            if msg.valid==1:
                status='streaming'
        return status
        
    def start_SVIN(self):
        msg=UBX_CFG_TMODE3()
        msg.encode(1,180, 20000)
        logger.info(f"RTK Streamer | Sending Survey-in start command to GPS")
        self.gpsp.send_to_gps(msg.serialize())
    
    def reset_gps(self, mode='cold'):
        if mode == 'cold':
            msg = UBX_RST_MSG_COLDSTART()
        elif mode =='warm':
            msg = UBX_RST_MSG_WARMSTART()
        elif mode == 'hot':
            msg = UBX_RST_MSG_HOTSTART()

        data = msg.serialize()
        logger.info(f"RTK Streamer | Sending RESET {mode} to GNS")
        self.gpsp.send_to_gps(data)
    
    def set_rate(self, rate):

        if rate != self.rate:
            self.rate = rate
            msg =UBX_CFG_RATE()
            msg.encode(rate)
            self.gpsp.send_to_gps(msg.serialize())


    def set_messages(self, mode):

        if mode == self.msg_mode:
            return
            #nothing to do msgs are already set
        self.msg_mode =mode
        
        gpsp = self.gpsp
                 
        obsolete_msgs={}
        required_msgs={}

        if mode == 'svin':           
            obsolete_msgs={
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
        "RTCM3.3-1005": b"\xF5\x05",
        "RTCM3.3-1074": b"\xF5\x4A",
        "RTCM3.3-1077": b"\xF5\x4D",
        "RTCM3.3-1084": b"\xF5\x54",
        "RTCM3.3-1087": b"\xF5\x57",
        "RTCM3.3-1230": b"\xF5\xE6"
        }

            required_msgs={
        "NAV-SVIN": b"\x01\x3B"
            }

        elif mode=='streaming':
            required_msgs={
        "NAV-SVIN": b"\x01\x3B",
        "RTCM3.3-1005": b"\xF5\x05",
        "RTCM3.3-1074": b"\xF5\x4A",
        "RTCM3.3-1077": b"\xF5\x4D",
        "RTCM3.3-1084": b"\xF5\x54",
        "RTCM3.3-1087": b"\xF5\x57",
        "RTCM3.3-1230": b"\xF5\xE6"
            }

        all_confirmed=0
        while not all_confirmed:
            for msg in obsolete_msgs:
                self.send_msg_deactivation_request(obsolete_msgs[msg])
            all_confirmed =1
        time.sleep(1)
        logger.info("RTK Streamer | Sending msg activation requests")

        all_confirmed=0
        while not all_confirmed:
            for msg in required_msgs:
                self.send_msg_activation_request(required_msgs[msg])
            all_confirmed =1
                

    def send_msg_deactivation_request(self,msgid):
        msg=UBX_CFG_MSG()
        msg.encode(msgid, UBX_PORT_NONE)
        self.gpsp.send_to_gps(msg.serialize())
    
    def send_msg_activation_request(self,msgid):
        msg=UBX_CFG_MSG()
        msg.encode(msgid, UBX_PORT_USB_ONLY)
        self.gpsp.send_to_gps(msg.serialize())
    
    def __del__(self):
        self.gpsp.stop()

def main():
    
    gpsp = GPSParser()
    rtk_streamer= RTKStreamer(gpsp)
    rtk_streamer.run()




main()