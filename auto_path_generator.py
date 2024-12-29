
##############################################created by miyawaki############################################################
#library imports
import time
from pymavlink import mavutil
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

#MAVLinkConnection class
class MAVLinkConnection:
    def __init__(self):
        self.connection = mavutil.mavlink_connection('tcp:localhost:14550')
        self.connection.wait_heartbeat()
        print("MAVLink bağlantısı başarılıyla yapıldı")
        #test için bir fonksiyon ekleyeceğim genel formatı göstermek adına alınan veriler
    def test_connection(self):
        important_messages = ['HEARTBEAT', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'VFR_HUD', 'GPS_RAW_INT', 'SYS_STATUS', 'BATTERY_STATUS']
        print("Testing connection...")
        for msg_type in important_messages:
            message = self.connection.recv_match(type=msg_type, blocking=True)
            print(f"{msg_type}: {message.to_dict()}" if message else f"No message for {msg_type}")
            
    def get_position(self):
        #anlık konum bilgilerini almak için
        msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000
            return (lat, lon, alt)
        else:
            print("No GLOBAL_POSITION_INT message received")
        return None
    

def main():
    mavlink_connection = MAVLinkConnection()
    mavlink_connection.test_connection()

if __name__ == "__main__":
    main()
