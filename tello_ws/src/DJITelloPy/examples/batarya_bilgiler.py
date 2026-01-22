from djitellopy import Tello
import time

def main():
    # Tello drone'u başlat
    tello = Tello()

    # Drone ile bağlantı kur
    tello.connect()

    # Batarya seviyesini al ve yazdır
    battery = tello.get_battery()
    print(f"Battery level: {battery}%")

    # Bağlantıyı sonlandır
    tello.end()

if __name__ == '__main__':
    main()

