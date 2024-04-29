import cv2 as cv
import numpy as np
import time

cap = cv.VideoCapture(2)
cv.namedWindow("frame", cv.WND_PROP_FULLSCREEN)
time_list = []

while True:
    start_time = time.time()

    ret, frame = cap.read()
    cv.imshow("frame", frame)

    time_list.append(1.0/(time.time()-start_time))

    if len(time_list) > 10000:
        time_list = time_list[1:]

    print(f'{np.mean(time_list):.1f}fps (std {np.std(time_list):.1f})')

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
