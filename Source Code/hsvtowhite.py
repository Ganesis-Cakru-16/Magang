import cv2
import numpy as np

def extract_white():
    # Buka file video
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Konversi dari BGR ke HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Tentukan range warna putih dalam HSV
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])

        # Buat mask untuk warna putih
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Hilangkan noise dengan operasi morfologi
        kernel = np.ones((5, 5), np.uint8)
        mask_white_open = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)

        # Gabungkan beberapa mask untuk menghilangkan bayangan
        mask_final = cv2.bitwise_and(mask_white, mask_white_open)

        # Aplikasikan mask ke frame asli
        res = cv2.bitwise_and(frame, frame, mask=mask_final)

        cv2.imshow('Original', frame)
        cv2.imshow('Only White', res)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Contoh penggunaan
extract_white()
