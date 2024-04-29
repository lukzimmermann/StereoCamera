import cv2 as cv
from src.stereo_camera import StereoCamera

cv.namedWindow("frame", cv.WND_PROP_FULLSCREEN)
stereo_camera = StereoCamera(0)

while True:
    image, _ = stereo_camera.get_splitted_images()
    cv.imshow("frame", image)


    if cv.waitKey(1) & 0xFF == ord('q'):
        break