
import cv2
import time
import sys
import argparse
import os
import obsrvrsnaps as o_snaps

def calibrate_camera(resolution=(800,480)):
	print("""Camera will now be calibrated. Follow the instructions on the screen.
Take about 20 snapshots, then press `q`""")
	input("Press any key to continue...\n")
	w, h = resolution
	o_snaps.save_snaps(width=w, height=h, folder="./snaps/")
	input("Snapshots saved. \nPress any key to generate transform matrix...")
	
	
