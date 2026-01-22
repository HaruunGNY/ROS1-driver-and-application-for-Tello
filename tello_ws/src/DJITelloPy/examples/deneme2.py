from djitellopy import Tello
import cv2
import time

def main():
    # Tello drone'u başlat
    tello = Tello()

    try:
        # Drone ile bağlantı kur
        tello.connect()

        # Batarya seviyesini kontrol et
        battery = tello.get_battery()
        print(f"Battery level: {battery}%")

        # Video akışını başlat
        print("Starting video stream...")
        tello.streamoff()  # Her ihtimale karşı önce kapat
        tello.streamon()   # Sonra başlat

        # Havalanma
        print("Taking off...")
        tello.takeoff()

        # 5 saniye havada bekle ve video akışını göster
        start_time = time.time()
        while time.time() - start_time < 5:
            # Video akışından bir kare al
            frame = tello.get_frame_read().frame

            # Kareyi ekrana göster
            cv2.imshow("Tello Camera", frame)

            # Pencereyi kapatmak için 'q' tuşuna bas
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # İniş
        print("Landing...")
        tello.land()

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Video akışını kapat ve kaynakları serbest bırak
        tello.streamoff()
        cv2.destroyAllWindows()
        tello.end()

if __name__ == '__main__':
    main()

