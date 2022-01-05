#! /usr/bin/env python3

from gpsparser import GPSParser
import serial
import sys
import logging
import time
from ubxhelper import *
import threading
import os
import argparse
import csv
import urllib.request as req
from UBXAssistOnline import UBXAssistOnline
import calendar
import datetime


logging.basicConfig(format='[%(levelname)8s]\t%(asctime)s: %(message)s ', filename='rtkstreamer.log', filemode='a', level=logging.INFO)
logger = logging.getLogger(__name__)
logger.level =logging.INFO

ANTENNA_FILE = "Antennas.loc"
ASSISTANCE_FILE="assistance_data.ubx"
ASSISTANCE_FILE=os.path.expanduser(ASSISTANCE_FILE)
LOCATION_FILE="HP_Antenna_Cypress.csv"
TIMEDIFFERENCE_FILE="timedifference.txt"
LATENCY= 0.093


class RTKStreamer():
    """RTK Streamer controls ublox GPS device via GPS Parser"""
    def __init__(self, gpsparser : GPSParser, mode='survey_in', survey_in="200,2.0", time_difference = 0, assistance_file = 0, location=(0,0,0,0)):
        self.gpsp = gpsparser
        self.status = 'undefined'
        self.fix_status = 'undefined'
        self.last_status=time.time()
        self.last_fix_status=time.time()
        self.survey_in = survey_in
        self.rate=0
        self.msg_mode=''
        self.location = location
        if location == (0,0,0,0):
            self.location_valid = 0
        else:
            self.location_valid = 1
        
        self.mode = mode
        logger.info(f'RTK Streamer | Starting in Mode {mode}')
        self.time_difference = time_difference
        self.assistance_file= assistance_file
        self.ublox_token=''
        self.keep_running = True
        self.t_assist = UBXAssistOnline(location, assistance_file) 
        self.time_differences=[]
        if self.assistance_file:
            self.t_assist.start()
            
    def run(self):
        self.gpsp.start()
        while(self.keep_running):
            self.wait_for_gps_ready()

            if self.mode == 'survey_in':
                if self.status == 'undefined':
                    self.reset_gps('hot')
                    self.msg_mode=''
                    self.rate=0
                    time.sleep(1)
                    self.set_rate(500)
                    self.set_messages('svin')
                    self.start_SVIN()
                    self.set_rate(500)
                    self.gpsp.udp_stream_active = False
                    time.sleep(2)
                elif self.status == 'surveying':
                    self.gpsp.udp_stream_active = False
                elif self.status == 'time':
                    self.set_rate(1000)
                    self.set_messages('time')
                    self.gpsp.udp_stream_active = True
            if self.mode=='fixed':
                if self.status == 'undefined':
                    self.reset_gps('hot')
                    self.msg_mode=''
                    self.rate=0
                    self.set_rate(1000)
                    self.set_messages('svin')
                    self.start_time_mode(self.location)
                    time.sleep(1)
                    self.gpsp.udp_stream_active = False
                elif self.status == 'time':
                    self.set_messages('time')
                    self.gpsp.udp_stream_active = True
            if self.mode== 'output_positions':
                if self.status=='streaming':
                    pass
                elif self.status=='acquiring':
                    if self.fix_status=='ok':
                        self.set_messages('output_positions')
                else:
                    self.reset_gps('hot')
                    self.msg_mode=''
                    self.stop_time_mode()
                    self.set_rate(1000)
                    self.set_messages('status')
                  
            self.process_ubx_messages()

    def set_status(self, status):
        if self.status == status:
            pass
        else:
            logger.info(f"RTK Streamer | Changing Status from {self.status} to {status}")
            self.status=status
        self.last_status = time.time()
    
    def process_ubx_messages(self):
        msg = self.gpsp.get_next_ubx_msg()
        while(msg):
            msg=msg.specify()

            if msg.msg_type == 'NAV-HPPOSLLH':
                if self.mode == 'output_positions':
                    self.set_status('streaming')
                    line=f"{msg.time_received}, {msg.lat:0.9f}, {msg.lon:.9f}, {msg.height:0.4f}\n"
                    with open(LOCATION_FILE,'a') as f:
                        f.write(line)

            if msg.msg_type == 'NAV-SVIN':
                logger.info(f"RTK Streamer | SVIN Status Dur: {msg.dur}s, Acc: {msg.mean_acc/10000:01.3f}m  Valid: {msg.valid}  Obs: {msg.num_obs}  In progress: {msg.in_progress}  itow: {msg.itow}, t_recv: {msg.time_received} ")
                if msg.in_progress==1:
                    self.set_status('surveying')
                    
            if msg.msg_type == 'NAV-STATUS':
                if msg.gpsfix == 5:
                    self.set_status('time')
                    
            if msg.msg_type == 'NAV-PVT':
                logger.debug(f"NAV-PVT | {msg.year}-{msg.month}-{msg.day} {msg.hour}:{msg.min}:{msg.sec+msg.nano*1e-9:11.9f} ")
                logger.debug(f"NAV-PVT | Validity Time-Date-fullyR-Mag {msg.validTime}-{msg.validDate}-{msg.fullyResolved}-{msg.validMag}")
                logger.debug(f"NAV-PVT | {msg.lat:.7f},{msg.lon:.7f},{msg.height:.3f} hAcc{msg.hAcc} vAcc:{msg.vAcc}")
                logger.debug(f"NAV-PVT | FixType: {msg.fixType} fixOk:{msg.gnssFixOk} invalidLLH:{msg.invalidLLH} ")
                
                fix_ok= msg.gnssFixOk & msg.validTime & msg.validDate &msg.fullyResolved
                if fix_ok:
                    self.fix_status='ok'
                    if self.assistance_file:
                        #update location for fix data
                        location=(msg.lat, msg.lon,msg.height, msg.hAcc)
                        self.t_assist.update_location(location)
                    if self.time_difference:
                        self.update_time_difference(msg)
                else:
                    self.fix_status='not ok'
                    if self.mode == 'output_positions':
                        self.set_status('acquiring')
                self.last_fix_status = time.time()

                if msg.fixType == 5:
                    self.set_status('time')


            msg = self.gpsp.get_next_ubx_msg()
            
        time_since_last_status = time.time() - self.last_status
        time_since_last_fix_status = time.time() - self.last_fix_status

        if time_since_last_status > 5:
            self.set_status('undefined')
            
        if time_since_last_fix_status > 5:
            self.fix_status = 'undefined'
        

    def update_time_difference(self, msg:UBX_NAV_PVT):
        timestamp_system=msg.time_received
        timestamp_gnss=calendar.timegm((msg.year,msg.month,msg.day,msg.hour,msg.min,msg.sec))+msg.nano*1e-9
        dt = int(((timestamp_gnss-timestamp_system)+LATENCY)*1e6)
        self.time_differences.append(dt)
        self.time_differences=self.time_differences[-20:]    
        dt_avg=sum(self.time_differences)/len(self.time_differences)*1e-6
        #line = f"{datetime.datetime.isoformat(datetime.datetime.now())}, {dt_avg:.6f}\n"
        line = f"{dt_avg:.6f}\n"
        
        with open(self.time_difference,'w') as f:
            f.write(line)

    def wait_for_gps_ready(self):
        while not self.gpsp.ready:
            time.sleep(0.2)
            
    def get_status(self):
        self.gpsp.ubx_buffer.clear()
        msg=self.gpsp.get_next_ubx_msg_type_timed('NAV-SVIN',5)
        status='undefined'
        if msg:
            logger.info(f"RTK Streamer | SVIN Status Dur: {msg.dur}s, Acc: {msg.mean_acc/10000:01.3f}m  Valid: {msg.valid}  Obs: {msg.num_obs}  In progress: {msg.in_progress}  itow: {msg.itow}, t_recv: {msg.t_recv} ")
            if msg.in_progress==1:
                status='surveying'
            if msg.valid==1:
                status='streaming'
        return status
        
    def start_SVIN(self):
        msg=UBX_CFG_TMODE3()
        
        survey_duration, survey_acc = self.survey_in.split(",")
        survey_duration = int(survey_duration)
        survey_acc = float(survey_acc)
        msg.encode_survey_in(survey_duration,survey_acc)
        logger.info(f"RTK Streamer | Sending Survey-in start command to GPS min Duration:{survey_duration} target Acc:{survey_acc:.3f}")
        self.gpsp.send_to_gps(msg.serialize())

    
    def start_time_mode(self, location):
        msg=UBX_CFG_TMODE3()
        (lat,lon, alt, acc)= location
        msg.encode_fixed(lat, lon, alt, acc)
        logger.info(f"RTK Streamer | Sending Start Time Mode -FIXED- command to GPS")
        self.gpsp.send_to_gps(msg.serialize())
    
    def stop_time_mode(self):
        msg=UBX_CFG_TMODE3()
        msg.encode_time_mode_off()
        logger.info(f"RTK Streamer | Sending TIME Mode OFF to GPS")
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
        "NAV-HPPOSLLH": b"\x01\x14",
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
        "NAV-SVIN": b"\x01\x3B",
        "NAV-STATUS": b"\x01\x03"
            }

        elif mode=='time':
            required_msgs={
        "NAV-STATUS": b"\x01\x03",
        "RTCM3.3-1005": b"\xF5\x05",
        "RTCM3.3-1074": b"\xF5\x4A", #GPS RTK MSM4
        #"RTCM3.3-1077": b"\xF5\x4D", #GPS RTK MSM7
        "RTCM3.3-1084": b"\xF5\x54", #GLONASS RTK MSM4
        #"RTCM3.3-1087": b"\xF5\x57", #GLONASS RTK MSM7
        "RTCM3.3-1230": b"\xF5\xE6"  #GLONASS RTK Code-phase bias
            }           
            obsolete_msgs={
        "NAV-SVIN": b"\x01\x3B",
        "NAV-HPPOSLLH": b"\x01\x14",
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
        "RTCM3.3-1077": b"\xF5\x4D",
        "RTCM3.3-1087": b"\xF5\x57",
        }
        
        elif mode=='output_positions':
            required_msgs={
        "NAV-HPPOSLLH": b"\x01\x14"
            }
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
        "RTCM3.3-1230": b"\xF5\xE6",
        "NAV-SVIN": b"\x01\x3B"
        }
        if mode=='status':        
            obsolete_msgs={
        "NAV-SVIN": b"\x01\x3B",
        "NAV-HPPOSLLH": b"\x01\x14",
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
        "RTCM3.3-1077": b"\xF5\x4D",
        "RTCM3.3-1087": b"\xF5\x57",
        } 
            required_msgs = {
            "NAV-PVT" : b"\x01\x07"
        }


        if self.time_difference or self.assistance_file:
            try:
                obsolete_msgs.pop('NAV-PVT')
            except KeyError:
                pass
            required_msgs['NAV-PVT']=b"\x01\x07"
        else:
            try:
                required_msgs.pop('NAV-PVT')
            except KeyError:
                pass
            obsolete_msgs['NAV-PVT']=b"\x01\x07"



            

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
    
    def stop(self):
        self.keep_running=False
        if self.t_assist.is_alive():
            self.t_assist.stop()
        if self.gpsp.is_alive():
            self.gpsp.stop()

    def __del__(self):
        self.gpsp.stop()

def get_location_from_file(location_name):
    with open (ANTENNA_FILE, 'r') as f:
        reader = csv.reader(f,delimiter=',')
        location=None
        for row in reader:
            if row[0] == location_name:
                lat= float(row[1])
                lon=float(row[2])
                height = float(row[3])
                acc=float(row[4])
                location =(lat,lon,height,acc)
                break
    if not location:
        raise RuntimeError(f"Antenna location with name {location_name} not found in {ANTENNA_FILE}")
    return location
        
def main():
    global LOCATION_FILE
    global ASSISTANCE_FILE
    global TIMEDIFFERENCE_FILE
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output_positions", help="output positions", nargs="?", const=LOCATION_FILE)
    parser.add_argument("-a", "--assistance_file", help="regulary update online assistance data", nargs="?", const=ASSISTANCE_FILE)
    parser.add_argument("-t", "--time_difference", help="regulary store difference to local time in file", nargs="?", const=TIMEDIFFERENCE_FILE)
    parser.add_argument("-s", "--survey_in", help="use position surveying, default mode",  nargs="?", const="200,2.0", default="180,2.0")
    parser.add_argument("-l", "--location", help="use fixed location for time mode and assistance data")
    #args=parser.parse_args(["-o","-l", "49.634584546, 8.631469629, 148.6396,1.000"])
    #args=parser.parse_args(["-a", "-l" , "HP","-t"])
    args=parser.parse_args()

    gpsp = GPSParser()
    
    streamer_mode='survey_in'
        
    if args.output_positions:
        LOCATION_FILE = args.output_positions
        streamer_mode='output_positions'
    
    streamer_location=(0,0,0,0)
    
    if args.time_difference:
        TIMEDIFFERENCE_FILE=args.time_difference
    
    if args.assistance_file:
        ASSISTANCE_FILE=args.assistance_file

    if args.location:
        if len(args.location.split(","))==4:
            lat,lon,height,acc= args.location.split(",")
            lat= float(lat)
            lon=float(lon)
            height = float(height)
            acc = float (acc)
            streamer_location = (lat,lon,height,acc)
            streamer_mode="fixed"
        else :
            try:
                streamer_location = get_location_from_file(args.location)
                streamer_mode="fixed"
            except RuntimeError as e:
                print(e)
                return

    rtk_streamer= RTKStreamer(gpsp, mode=streamer_mode, survey_in=args.survey_in, time_difference=args.time_difference, assistance_file=args.assistance_file, location=streamer_location)
    try: 
        rtk_streamer.run()
    except KeyboardInterrupt:
        rtk_streamer.stop()



main()