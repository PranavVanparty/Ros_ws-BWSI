import cv2
import numpy as np
import glob 

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

len_width = (9, 6)  # Number of inner corners per chessboard row and column
objp = np.zeros((len_width[0] * len_width[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:len_width[0, 0:len_width[1]]].T.reshape(-1, 2)


objectPoints = []
imagePoints = []

frames = glob.glob('calibration_images/*.jpg')

for frame in frames:
    img = cv2.imread(frame)
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(grey, len_width, None)
    
    if ret :
        objectPoints.append(objp)
        corners2 = cv2.cornerSubPix(grey, corners, (11,11), (-1,-1), criteria)
        imagePoints.append(corners2)
        cv2.drawChessboardCorners(img, len_width, corners2, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(500)
        
cv2.destroyAllWindows()
    