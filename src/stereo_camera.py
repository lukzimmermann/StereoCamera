import cv2 as cv
import numpy as np

class StereoCamera():
    def __init__(self, device_id: int) -> None:
        self.capture = cv.VideoCapture(device_id)

    def get_raw_image(self) -> np.ndarray:
        ret, frame = self.capture.read()
        if ret:
            return frame
        else:
            raise Exception("No valid image")


    def get_splitted_images(self) -> tuple[np.ndarray,np.ndarray]:
        ret, frame = self.capture.read()
        if ret:
            shape = frame.shape
            left_frame = frame[:,0:shape[1]//2,:]
            right_frame = frame[:,shape[1]//2:,:]
            return left_frame, right_frame
        else:
            raise Exception("No valid image")