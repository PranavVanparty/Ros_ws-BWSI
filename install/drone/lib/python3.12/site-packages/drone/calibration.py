import cv2
from matplotlib import pyplot as plt
import numpy as np
import glob 

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

len_width = (5, 8)  # Number of inner corners per chessboard row and column (CORRECTED from 9,6)
objp = np.zeros((len_width[0] * len_width[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:len_width[0], 0:len_width[1]].T.reshape(-1, 2)

objectPoints = []
imagePoints = []

frames = glob.glob('/home/pranav/ros2_ws/src/drone/calibration_imgs/*.png')
# Filter out the previously generated detected_*.png files to avoid double-processing
frames = [f for f in frames if not 'detected_' in f]
print(f"Found {len(frames)} calibration images")

successful_detections = 0

for i, frame in enumerate(frames):
    print(f"\nProcessing image {i+1}/{len(frames)}: {frame}")
    img = cv2.imread(frame)
    
    if img is None:
        print(f"Could not load image: {frame}")
        continue
        
    print(f"Image shape: {img.shape}")
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Show the image for debugging
    cv2.imshow("Original", img)
    
    # Try to find chessboard corners
    ret, corners = cv2.findChessboardCorners(grey, len_width, None)
    
    print(f"Chessboard detection result: {ret}")
    
    if ret:
        print("SUCCESS: Found chessboard corners!")
        successful_detections += 1
        objectPoints.append(objp)
        corners2 = cv2.cornerSubPix(grey, corners, (11,11), (-1,-1), criteria)
        imagePoints.append(corners2)
        
        # Draw the corners
        img_with_corners = img.copy()
        cv2.drawChessboardCorners(img_with_corners, len_width, corners2, ret)
        cv2.imshow("Detected Corners", img_with_corners)
        print("Hiii - Chessboard detected successfully!")
        
        # Wait for key press to continue
        key = cv2.waitKey(2000)  # Wait 2 seconds or until key press
        if key == ord('q'):
            break
    else:
        print("FAILED: Could not find chessboard corners")
        # Wait briefly to see the image
        cv2.waitKey(1000)  # Wait 1 second

print(f"\nSummary: Successfully detected chessboards in {successful_detections}/{len(frames)} images")

# Add camera calibration if we have enough detections
if successful_detections >= 10:
    print("You have enough images for calibration!")
    # Perform camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objectPoints, imagePoints, grey.shape[::-1], None, None)
    
    if ret:
        print("\nCamera calibration successful!")
        print("Camera Matrix:")
        print(camera_matrix)
        print("\nDistortion Coefficients:")
        print(dist_coeffs)
        
    else:
        print("Camera calibration failed!")
else:
    print(f"Need at least 10 successful detections for good calibration. You have {successful_detections}.")
        
cv2.destroyAllWindows()
    