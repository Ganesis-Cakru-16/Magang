import cv2
import numpy as np

def green():
    # Open the video file
    cap = cv2.VideoCapture(0)
    success, image =cap.read()

    while success:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Tentukan range warna putih dalam HSV
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 60, 255])

        # Buat mask untuk warna putih
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Hilangkan noise dengan operasi morfologi
        kernel = np.ones((5, 5), np.uint8)
        mask_white_open = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)

        # Gabungkan beberapa mask untuk menghilangkan bayangan
        mask_final = cv2.bitwise_and(mask_white, mask_white_open)

        # Aplikasikan mask ke frame asli
        res = cv2.resize(cv2.bitwise_and(frame, frame, mask=mask_final),(1280,720))

        cv2.imshow('Original', frame)
        cv2.imshow('Only White', res)

        tl =[306,399] 
        bl=[0,720]
        tr=[976,399]
        br=[1280,720]

        #cv2.circle(img,tl,5,[0,0,255],-1)
        #cv2.circle(img,bl,5,[0,0,255],-1)
        #cv2.circle(img,tr,5,[0,0,255],-1)
        #cv2.circle(img,br,5,[0,0,255],-1)

        pts1=np.float32([tl,bl,tr,br])
        pts2=np.float32([[0,0],[0,720],[1280,0],[1280,720]]) 

        matrix=cv2.getPerspectiveTransform(pts1,pts2)
        newframe=cv2.warpPerspective( res ,matrix, (1280,720))
        resize=cv2.resize(newframe,(720,637))
        small=cv2.resize(resize,(26,23))
        reenlarge=cv2.resize(small,(720,637))
        cv2.imshow("BEV",newframe)
        cv2.imshow("pizels",reenlarge)
        array = np.zeros((26,23),dtype=np.uint8)
        for y in range(23):
            for x in range(26):
                # Set pixel color at (x, y) to a specific color (for example, white)
                array[x,y]=0 # Set pixel to white (255, 255, 255) in BGR

        # Access individual pixel values
        for y in range(23):
            for x in range(26):
                # Access pixel color at (x, y)
                array [x,y] = small[y, x,0]
        print(array)
        if cv2.waitKey(1)==27:
            break

    cap.release()
    cv2.destroyAllWindows()

# Example usage
green()