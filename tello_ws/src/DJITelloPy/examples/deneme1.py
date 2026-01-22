from djitellopy import Tello
import time

def test():
    tello = Tello()

    try:
        # Bağlantıyı başlat
        tello.connect()
        print('Connected to drone')

        # Havalanma
        tello.takeoff()
        print('Drone has taken off')

        # 5 saniye havada kal
        time.sleep(5)

        # İniş
        tello.land()
        print('Drone has landed')

    except Exception as e:
        print(f"Error: {e}")
    finally:
        tello.quit()

if __name__ == '__main__':
    test()

