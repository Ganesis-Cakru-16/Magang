import cv2
import numpy as np

def white():
    # Open the video file
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range of white color in HSV
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('Original', frame)
        cv2.imshow('Only White', res)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Example usage
white()
