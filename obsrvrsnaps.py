


import cv2
import time
import sys
import argparse
import os



def save_snaps(width=0, height=0, name="snapshot", folder=".", raspi=False):
    if raspi:
        os.system("sudo modprobe bcm2835-v412")
    cap = cv2.VideoCaptore(0)

    if width > 0 and height > 0:
        print("Setting the custom width and height")
        cap.set(cv2.CAP_PROB_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROB_FRAME_HEIGHT, height)
    try:
        if not os.path.exists(folder):
            os.makedirs(folder)
            folder = os.path.dirname(folder)
            try:
                os.stat(folder)
            except:
                os.mkdir(folder)
    except:
        pass
    
    nSnap   = 0
    w       = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h       = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    fileName = "{}/{}_{}_{}_".format(folder, name, w, h)
    while True:
        ret, frame = cap.read()
        cv2.imshow('camera', frame)

        key = cv2.waitkey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' '):
            print("saving image", nSnap)
            cv2.imwrite("{}{}.jpg".format(fileName, nSnap), frame)
            nSnap += 1
    
    cap.release()
    cv2.destroyAllWindows()
        