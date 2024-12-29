
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
    
    #mod değişikliği ve göreve hazırlık
    def set_guided_mode(self):
        mode = 'GUIDED'
        mode_id = self.connection.mode_mapping()[mode]
        
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = self.connection.recv_match(type=['HEARTBEAT'], blocking=True)
            #hearbeat dediğimiz tcp üzerinden hava aracının canlı verilerinin alınmasının kontrolü
            if msg.custom_mode == mode_id:
                print("GUIDED moda geçiş başarılı!")
                return True
            time.sleep(0.1)
        return False
    
    def clear_waypoints(self):
        #Mevcut waypoint'leri temizle
        self.connection.mav.mission_clear_all_send(
            self.connection.target_system,
            self.connection.target_component)
        self.connection.recv_match(type=['MISSION_ACK'], blocking=True)

    def upload_waypoints(self, waypoints):
        #waypointleri geri yükleme
        self.clear_waypoints()
        
        self.connection.mav.mission_count_send(
            self.connection.target_system,
            self.connection.target_component,
            len(waypoints))
        
        for i, wp in enumerate(waypoints):#ennumerate ile itarasyon yapıyoruz
            #ihanın isterlerine özellikleirne uygun dönüş yarıçağı vs paremetrelerin girilmesi
            self.connection.mav.mission_item_send(
                self.connection.target_system,
                self.connection.target_component,
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,
                2,  # Acceptance radius
                5,  # Pass radius
                0,  # Yaw
                0,  # Empty
                wp[0], wp[1], wp[2])
            
            self.connection.recv_match(type=['MISSION_REQUEST'], blocking=True)
        
        self.connection.recv_match(type=['MISSION_ACK'], blocking=True)
        print(f"{len(waypoints)} waypoint yüklendi")
    
    
    def start_mission(self):
        #görev başlatma
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0)
        self.connection.recv_match(type=['COMMAND_ACK'], blocking=True)
        print("Misyon başlatıldı")
        
class FlightPathCalculator:
    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        #iki nokta arası mesafe hesaplama
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        #iki nokta arası yön hesaplama
        lat1, lon1 = map(math.radians, [lat1, lon1])
        lat2, lon2 = map(math.radians, [lat2, lon2])

        y = math.sin(lon2-lon1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - \
            math.sin(lat1) * math.cos(lat2) * math.cos(lon2-lon1)
        
        return math.degrees(math.atan2(y, x))

    @staticmethod
    def calculate_waypoints(start_pos, target_pos):
        #Başlangıç ve hedef noktalar arasında ara waypointler
        waypoints = [start_pos] 
        
        total_distance = FlightPathCalculator.haversine_distance(
            start_pos[0], start_pos[1], target_pos[0], target_pos[1])
        
        # Bearing hesapla
        bearing = FlightPathCalculator.calculate_bearing(
            start_pos[0], start_pos[1], target_pos[0], target_pos[1])
        
        # 3 ara nokta oluştur
        for i in range(1, 4):
            fraction = i / 4  #bu kısım daha anlamlı matematiksel olabilir
            
            # Ara nokta koordinatlarını hesapla
            lat1 = math.radians(start_pos[0])
            lon1 = math.radians(start_pos[1])
            ang_dist = (total_distance * fraction) / 6371000

            lat2 = math.asin(math.sin(lat1) * math.cos(ang_dist) + 
                           math.cos(lat1) * math.sin(ang_dist) * math.cos(math.radians(bearing)))
            
            lon2 = lon1 + math.atan2(math.sin(math.radians(bearing)) * math.sin(ang_dist) * math.cos(lat1),
                                    math.cos(ang_dist) - math.sin(lat1) * math.sin(lat2))
            
            # Yükseklik interpolasyonu
            alt = start_pos[2] + (target_pos[2] - start_pos[2]) * fraction
            
            waypoint = (math.degrees(lat2), math.degrees(lon2), alt)
            waypoints.append(waypoint)
        
        waypoints.append(target_pos)  # Hedef nokta
        return waypoints    
    
class FlightVisualizer:
    def __init__(self):
        plt.ion()
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.actual_path = []

    def update_plot(self, current_pos, waypoints):
        self.ax.clear()
        
        # Waypoint'leri birleştiren çizgi
        lats, lons, alts = zip(*waypoints)
        self.ax.plot(lons, lats, alts, 'r--', label='Planlanan Rota')
        
        # Waypoint'leri işaretle
        for i, wp in enumerate(waypoints):
            if i == 0:
                label = 'Başlangıç'
                color = 'green'
            elif i == len(waypoints) - 1:
                label = 'Hedef'
                color = 'red'
            else:
                label = f'WP{i}'
                color = 'orange'
            
            self.ax.scatter(wp[1], wp[0], wp[2], 
                          color=color, s=100, label=label)
        
        # Gerçek rotayı çiz
        if self.actual_path:
            act_lats, act_lons, act_alts = zip(*self.actual_path)
            self.ax.plot(act_lons, act_lats, act_alts, 'b-', label='Gerçek Rota')
        
        # Mevcut konumu işaretle
        if current_pos:
            self.ax.scatter(current_pos[1], current_pos[0], current_pos[2], 
                          color='blue', s=100, label='Mevcut Konum')
        
        self.ax.set_xlabel('Boylam')
        self.ax.set_ylabel('Enlem')
        self.ax.set_zlabel('Yükseklik (m)')
        self.ax.legend()
        
        plt.draw()
        plt.pause(0.1)

class KozmosUAVControl:
    def __init__(self):
        self.mavlink = MAVLinkConnection()
        self.calculator = FlightPathCalculator()
        self.visualizer = FlightVisualizer()

    def navigate_to_target(self, target_lat, target_lon, target_alt):
        current_pos = self.mavlink.get_position()
        if not current_pos:
            print("Konum alınamadı!")
            return

        target_pos = (target_lat, target_lon, target_alt)
        print(f"\nMevcut konum: Lat={current_pos[0]:.6f}, Lon={current_pos[1]:.6f}, Alt={current_pos[2]:.1f}m")
        print(f"Hedef konum: Lat={target_lat:.6f}, Lon={target_lon:.6f}, Alt={target_alt:.1f}m")

        if not self.mavlink.set_guided_mode():
            print("GUIDED moda geçilemedi!")
            return

        # Waypoint'leri hesaplama
        waypoints = self.calculator.calculate_waypoints(current_pos, target_pos)
        print(f"Waypoint'ler hesaplandı: {len(waypoints)} nokta")

        # Waypoint'leri yükleme ve başlama
        self.mavlink.upload_waypoints(waypoints)
        self.mavlink.start_mission()

        # Uçuşu takip etme
        while True:
            current = self.mavlink.get_position()
            if current:
                self.visualizer.actual_path.append(current)
                self.visualizer.update_plot(current, waypoints)
                
                dist = self.calculator.haversine_distance(
                    current[0], current[1], target_lat, target_lon)
                
                print(f"\rKonum: Lat={current[0]:.6f}, Lon={current[1]:.6f}, " + 
                      f"Alt={current[2]:.1f}m, Mesafe={dist:.1f}m", end='')
                
                if dist < 5:
                    print("\nHedefe ulaşıldı!")
                    break
            
            time.sleep(0.1)
def main():
    mavlink_connection = MAVLinkConnection()
    mavlink_connection.test_connection()

if __name__ == "__main__":
    main()
