
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
        for i in range(3): # 3 kere veri alıyoruz
            msg = self.connection.recv_match(blocking=True)
            print(f"Veri {i+1}: {msg.get_type()} | {msg.to_dict()}")


def main():
    print("Kozmos İHA Kontrol Sistemi")
    connection = MAVLinkConnection()
    connection.test_connection()

if __name__ == "__main__":
    main()
