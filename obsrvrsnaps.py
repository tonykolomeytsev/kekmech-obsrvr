


import cv2
import time
import sys
import argparse
import os



def save_snaps(width=0, height=0, name="snapshot", folder=".", raspi=False):
    cv2.CAP_PROP_FRAME_WIDTH = 3
    cv2.CAP_PROP_FRAME_HEIGHT = 4
    if raspi:
        os.system("sudo modprobe bcm2835-v412")
    cap = cv2.VideoCapture(0)

    if width > 0 and height > 0:
        print("Setting the custom width and height: {}x{}".format(width,height))
        cap.set(3, width)
        cap.set(4, height)
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
    w       = cap.get(3)
    h       = cap.get(4)

    fileName = "{}/{}_{}_{}_".format(folder, name, int(w), int(h))
    print("Press Whitespace to make shapshot, press `q` when complete.")
    while True:
        time.sleep(0.2)
        ret, frame = cap.read()
        cv2.imshow('camera', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' '):
            img_filename = "{}{}.jpg".format(fileName, nSnap)
            print("Saving image {}".format(img_filename), nSnap)
            cv2.imwrite(img_filename, frame)
            nSnap += 1
    
    cap.release()
    cv2.destroyAllWindows()


