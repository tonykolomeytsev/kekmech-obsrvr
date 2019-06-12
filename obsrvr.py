
import time, sys, argparse, os, math
import cv2
import cv2.aruco as aruco
import obsrvrsnaps
import numpy as np



# utils
def calibrate_camera(resolution=(800,480)):
	print("""Camera will now be calibrated. Follow the instructions on the screen.
Take about 20 snapshots, then press `q`""")
	input("Press any key to continue...\n")
	w, h = resolution
	obsrvrsnaps.save_snaps(width=w, height=h, folder="./snaps/")
	input("Snapshots saved. \nPress any key to generate transform matrix...")
	obsrvrsnaps.generate_matrix(workingFolder="./snaps/")
	
# define tag
id_to_kind = 72
marker_size = 6 # [cm]

# get the camera calibration matrices
calib_path = "./snaps/"
camera_matrix = np.loadtxt(calib_path+'camera_matrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path+'camera_distortion.txt',delimiter=',')

# 180 deg rotation matrix around the X axis
R_flip = np.zeros(shape=(3,3),dtype=np.float32)
R_flip[0,0]=1.0
R_flip[1,1]=-1.0
R_flip[2,2]=-1.0

# define aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
parameters = aruco.DetectorParameters_create()

# capture camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
	time.sleep(0.125)
	#read frame
	ret, frame = cap.read()

	#convert in gray scale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	#find all the aruco markers in image
	corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,cameraMatrix=camera_matrix, distCoeff=camera_distortion)
	

	if not ids is None: # and ids[0] == id_to_kind:
		# ret = [rvec, tvec, ?]
		# array of rotation and position of each marker in camera frame
		# rvec = [[rvec_1], [rvec_2], ...] rotation vector
		# tvec = [[tvec_1], [tvec_2], ...] translation vector
		ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
		
		ids = ids.reshape(-1)
		print(ids)
		for i in range(len(ids)):
			# unpack the output, get only the first
			rvec, tvec = ret[0][i,0,:], ret[1][i,0,:]

			#draw the detected marker and put a reference frame over it
			aruco.drawDetectedMarkers(frame, corners)
			aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

	#display frame
	cv2.imshow('frame', frame)

	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		cap.release()
		cv2.destroyAllWindows()
		break
