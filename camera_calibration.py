import numpy as np
import cv2
import glob
import os
import sys
from src.stereo_camera import StereoCamera
import time

def camera_calibration(side: str):
    # checkerboard dimensions
    cbrow = 9
    cbcol = 7

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((cbrow*cbcol,3), np.float32)
    objp[:,:2] = np.mgrid[0:cbcol,0:cbrow].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.


    def findCorners(gray_, img_, fname_):
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray_, (cbcol,cbrow),None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray_,corners,(11,11),(-1,-1),criteria)
            print ("added corners: ", len(corners2))
            imgpoints.append(corners2)
            # Draw and display the corners
            img_ = cv2.drawChessboardCorners(img_, (cbcol,cbrow), corners2,ret)
            cv2.imshow('img',img_)
            cv2.waitKey(500)

    # use previously recorded images

    images = glob.glob(f'camera_calibration/{side}/*.png')

    print(images)
    for fname in images:
        print (fname)
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        findCorners(gray, img, fname)

    cv2.destroyAllWindows()

    #camera calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    # Saving the calibration
    cv_file = cv2.FileStorage(f"calib_{side}.yaml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    cv_file.release()

    print("camera_matrix:\n ", mtx)
    print("dist_matrix:\n ", dist)



def main(camera_side):
    if not os.path.exists(f'camera_calibration/{camera_side}'):
        os.mkdir(f'camera_calibration/{camera_side}')
    TIME_INTERVAL = 5
    NUMBER_OF_IMAGES = 10

    stereo_camera = StereoCamera(0)

    cv2.namedWindow("frame", cv2.WND_PROP_FULLSCREEN)

    captured_images = 0
    target_time = time.time() + TIME_INTERVAL
    
    while captured_images < NUMBER_OF_IMAGES:
        left, right = stereo_camera.get_splitted_images()
        image = None

        if camera_side == 'left':
            image = left
        else:
            image = right

        remaining_time = np.abs(target_time-time.time())
        image = cv2.putText(image.copy(), str(f'{remaining_time:.1f}'), (20, 80), cv2.FONT_HERSHEY_SIMPLEX,  
           3, (0,0,255), 3, cv2.LINE_AA)
        cv2.imshow("frame", image)

        if time.time() > target_time:
            print(f'{captured_images}. Image recorded')
            cv2.imshow("frame", image)
            cv2.imwrite(f'camera_calibration/{camera_side}/{captured_images}.png', image)
            cv2.waitKey(500)
            target_time = time.time() + TIME_INTERVAL
            captured_images += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("start calculation camera matrix...")
    
    camera_calibration(camera_side)

    print("finished!")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No side in parameter...")
        sys.exit(1)

    camera_side = sys.argv[1]
    main(camera_side)