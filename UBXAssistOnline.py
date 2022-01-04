#! /usr/bin/env python
import urllib.request as req
import threading
import time
import os


import logging
logger = logging.getLogger(__name__)


token_file = "~/.keys/ublox_token.txt"
token_file=os.path.expanduser(token_file)
ASSISTANCE_FILE="assistance_data.ubx"
ASSISTANCE_FILE=os.path.expanduser(ASSISTANCE_FILE)

class UBXAssistOnline(threading.Thread):
    def __init__(self, location):
        self.lat ,self.lon, self.alt,self.acc = location
        self.assistance_data = b''
        self.keep_running = True
        self.data = b''
        self.token = ''
        try:
            with open(token_file, 'r') as f:
                self.token = f.read()
            self.token=self.token.replace('\n','')
            self.token=self.token.replace('\r','')
        except Exception as e:
            logger.error(f"UBXAssistOnline | {e}")
            self.keep_running=False
        threading.Thread.__init__(self)

    def run(self):
        interval = 600  # 10 minutes
        last_update = 0
        update_due = 1
        while(self.keep_running):

            now = time.time()
            if (now > last_update+interval):
                update_due = 1
            else:
                update_due = 0

            if update_due:
                self.update_assistance_data()
                last_update=now
            
            time.sleep(1)
                
    def update_assistance_data(self):
        data = None
        if self.lat and self.lon and self.alt:
            url = f"http://online-live1.services.u-blox.com/GetOnlineData.ashx?token={self.token};gnss=gps,glo;datatype=eph;lat={self.lat:.6f};lon={self.lon:.6f};alt={self.alt:.6f};pacc={self.acc:.6f};filteronpos"
        else:
            url = f"http://online-live1.services.u-blox.com/GetOnlineData.ashx?token={self.token};gnss=gps,glo;datatype=eph"

        try:
            data = req.urlopen(url).read()
        except:
            data = None

        if data:
            data = self.remove_mga_ini_msg(data)
            self.data = data
            self.write_data_to_file()
        else:
            self.load_data_from_file()

    def remove_mga_ini_msg(self, data):
        while (data[:6] == b'\xb5\x62\x13\x40\x18\x00'):
            data = data[32:]
        if data[:3] != b'\xb5\x62\x13':
            data = None
        return data

    def load_data_from_file(self):
        try:
            f = open(ASSISTANCE_FILE, 'rb')
            self.data = f.read()
            f.close()
        except FileNotFoundError:
            pass

    def write_data_to_file(self):
        logger.info(f"UBXAssistOnline | Writing {len(self.data)} bytes Assistance Data to file {ASSISTANCE_FILE}")
        try:
            f = open(ASSISTANCE_FILE, 'wb')
            f.write(self.data)
        finally:
            f.close()

    def stop(self):
        self.keep_running = False
        logger.info("UBXAssistOnline | Stopping Thread")
    
    def __del__(self):
        logger.info("UBXAssistOnline | Deleting Thread object")


if __name__ == "__main__":
    t = UBXAssistOnline()
    try:
        t.start()
        t.join()
    except KeyboardInterrupt:
        t.stop()
        time.sleep(1)
