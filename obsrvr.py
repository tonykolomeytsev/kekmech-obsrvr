from picamera.array import PiRGBArray
import picamera
import cv2
import numpy as np
points = []

def click(event, x, y, flags, param):
	global points
	if (len(points) == 4) and (event == cv2.EVENT_LBUTTONDOWN):
		points = []
		print(points)
	if event == cv2.EVENT_LBUTTONDOWN:
		points.append((x, y))
		print(points)
	if event == cv2.EVENT_RBUTTONDOWN:
		points = []
		print(points)
	
def init_camera():
	cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
	cv2.setMouseCallback("Frame", click)
	cv2.resizeWindow("Frame", 320, 240)
	
	with picamera.PiCamera() as camera:
		camera.resolution =(640, 480)
		camera.framerate = 32
		rawCapture = PiRGBArray(camera, size=(640, 480))
		for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
			frame = frame.array
			rawCapture.truncate(0)
			cv2.imshow("Frame", frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
	cv2.destroyAllWindows()
		##frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	return np.array(points)
