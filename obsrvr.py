
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


def isRotationMatrix(R):
	Rt = np.transpose(R)
	sbi = np.dot(Rt, R)
	I = np.identity(3, dtype=R.dtype)
	n = np.linalg.norm(I - sbi)
	return n < 1e-6

def rotToEuler(R):
	assert(isRotationMatrix(R))
	sy=math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
	singular = sy < 1e-6
	
	if not singular:
		x=math.atan2(R[2,1],R[2,2])
		y=math.atan2(-R[2,0],sy)
		z=math.atan2(R[1,0],R[0,0])
	else:
		x=math.atan2(-R[1,2],R[1,1])
		y=math.atan2(-R[2,0],sy)
		z=0
	
	return np.array([x,y,z])


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

font = cv2.FONT_HERSHEY_PLAIN

while True:
	time.sleep(0.1)
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
		#print(ids)
		for i in range(len(ids)):
			# unpack the output, get only the first
			rvec, tvec = ret[0][i,0,:], ret[1][i,0,:]
			
			#draw the detected marker and put a reference frame over it
			aruco.drawDetectedMarkers(frame, corners)
			aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
			
			#print tag position respect to camera frame
			str_pos = "pos: x=%4.0f   y=%4.0f   z=%4.0f"%(tvec[0],tvec[1],tvec[2])
			cv2.putText(frame, str_pos, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
			
			# rot matrix (tag-> camera)
			R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
			R_tc = R_ct.T
			
			#get euler angles (nneds to be flipped)
			roll_marker, pitch_marker, yaw_marker = rotToEuler(R_flip*R_tc)
			
			#print marker's attitude respect to camera frame
			str_att = "eul: roll=%4.0f   pitch=%4.0f   yaw=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),math.degrees(yaw_marker))
			cv2.putText(frame, str_att, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
			
			#get position and attitude of the camera respect to the marker
			pos_camera = -R_tc*np.transpose(np.matrix(tvec))
			roll_camera,pitch_camera, yaw_camera = rotToEuler(R_flip*R_tc)

			str_pos = "Cam pos: x=%4.0f   y=%4.0f   z=%4.0f"%(pos_camera[0],pos_camera[1],pos_camera[2])
			cv2.putText(frame, str_pos, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
			str_att = "Cam eul: roll=%4.0f   pitch=%4.0f   yaw=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),math.degrees(yaw_camera))
			cv2.putText(frame, str_pos, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

	#display frame
	cv2.imshow('frame', frame)

	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		cap.release()
		cv2.destroyAllWindows()
		break
