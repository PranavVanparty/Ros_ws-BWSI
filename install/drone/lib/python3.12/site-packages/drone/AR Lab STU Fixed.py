# import cv2 as cv
# import numpy as np
# from std_msgs.msg import Int32MultiArray
# import csv
# import os


# def find_relative_pose(pic):
#         ids = Int32MultiArray() #Each AR tag has a number written on it and a unique color combo, this will contain info for each 
#         arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
        
#         mtx = #intrinsic camera matrix
#         distortion = #camera distortion
#         #Find the matrix and distortion with https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
#         #It's unique to your camera and found experimentally


#         tag_info = grab_tag(#find and process the image)
#         #grab_tag is defined below


#         if len(tag_info)>1:#if AR tag is found
#             rvec, tvec = tag_info
#             trans, orien = find_pos(#get position relative to AR tag)
#             print(f"Orientation: {orien}, Position: {trans}")
#         else:
#             print('not found')
        
# def grab_tag(#ADD):
#         """This function extracts the corners of the AR tag from the image"""


#         # Preprocess the image
#         if tag is None:
#             print("Error: Image is None.")
#             return []
#         tag = #load in tag in grayscale
#         #Do other pre processing (optional)
        

#         corners, ids, rejects = #find corners of the AR tag
#         # https://docs.opencv.org/4.x/d9/d6a/group__aruco.html#gaba7f1e107f93451e2bc43b8ea96eef8c

#         if len(corners) == 0:
#             print('No corners found')
#             return []
        
#         rvec, tvec = my_estimatePoseSingleMarkers2(#get stats from AR tag)
#         #rvec=rodrigues vector, tvec=translation vector of the AR tag in the camera frame

#         cv.drawFrameAxes(image, mtx, distortion, rvec, tvec, length=10)#optional, helps visualization
#         #red=x, y=green, z=blue


#         print(f"rvecs: {rvec}, tvecs: {tvec}\n")
#         return rvec, tvec
        
# def find_pos(rvec, tvec):
#         """rvec is a rodriguez vector and tvec is the position of the tag relative to
#         the camera in camera frame. Use rvec to create a rotation matrix and find
#         position of the drone relative to the tag; return the position of the drone. 
#         Additionally, it may be helpful to have the orientation of the drone so you can could
#         also return euler angles, the full rotation matrix, or some other information. This is just
#         some math so I'll let you do the whole thing yourselves"""
        
#         # Create rotation matrix from rvec
#         # rot_mat = cv2.Rodrigues(rvec)
        
        
        
        
#         return drone_from_ar, orientation

# def my_estimatePoseSingleMarkers2(corners, mtx, distortion, marker_size=26.6): # 26.6cm is side length of AR tag
#         """This function gets the rvec and tvec """

#         c= np.array(corners[0])[0]#corners of the AR tag in a numpy array in camera frame
#         # https://github.com/Menginventor/aruco_example_cv_4.8.0/blob/main/pose_estimate.py


#         marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],#corners of AR in world frame
#                                 [marker_size / 2, marker_size / 2, 0],
#                                 [marker_size / 2, -marker_size / 2, 0],
#                                 [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

#         _, R, t = cv.solvePnP(#ADD)
#         return R, t

# #Read image into CV2
# path = os.path.expanduser('ADD PATHNAME')
# image = cv.imread(path, cv.IMREAD_COLOR)
# if image is None:
#     print("Failed to load image. Check the path and file.")

# find_relative_pose(image)#compute world frame position and orientation

# #optional debugging
# cv.imshow("Pose Debug", image)
# cv.waitKey(0)
# cv.destroyAllWindows()