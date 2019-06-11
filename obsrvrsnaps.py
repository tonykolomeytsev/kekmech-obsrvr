import cv2
import time
import sys
import argparse
import os
import numpy as np
import glob



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
            print("Saving image {}".format(img_filename))
            cv2.imwrite(img_filename, frame)
            nSnap += 1
    
    cap.release()
    cv2.destroyAllWindows()



## dimension must be INT only
def generate_matrix(nRows=9, nCols=6, dimension=23, workingFolder=".", imageType="jpg"):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, dimension, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((nRows*nCols,3), np.float32)
    objp[:,:2] = np.mgrid[0:nCols,0:nRows].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    print("\nThe script will look for every image in the provided folder and will show the pattern found." \
          "User can skip the image pressing ESC or accepting the image with RETURN. " \
          "At the end the end the following files are created:" \
          "  - camera_distortion.txt" \
          "  - camera_matrix.txt \n\n")
    # Find the images files
    filename    = workingFolder + "/*." + imageType
    images      = glob.glob(filename)

    if len(images) < 9:
        print("Not enough images were found: at least 9 shall be provided!!!")
        sys.exit()

    else:
        nPatternFound = 0
        imgNotGood = images[1]
        for fname in images:
            if 'calibresult' in fname: continue
            #-- Read the file and convert in greyscale
            img     = cv2.imread(fname)
            gray    = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            print("Reading image ", fname)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (nCols,nRows),None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                print("Pattern found! Press ESC to skip or ENTER to accept")
                #--- Sometimes, Harris cornes fails with crappy pictures, so
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                # Draw and display the corners
                cv2.drawChessboardCorners(img, (nCols,nRows), corners2,ret)
                cv2.imshow('img',img)
                # cv2.waitKey(0)
                k = cv2.waitKey(0) & 0xFF
                if k == 27: #-- ESC Button
                    print("Image Skipped")
                    imgNotGood = fname
                    continue
                print("Image accepted")
                nPatternFound += 1
                objpoints.append(objp)
                imgpoints.append(corners2)
                # cv2.waitKey(0)
            else:
                imgNotGood = fname
    
    cv2.destroyAllWindows()

    if (nPatternFound > 1):
        print("Found {} good images".format(nPatternFound))
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        # Undistort an image
        img = cv2.imread(imgNotGood)
        h,  w = img.shape[:2]
        print("Image to undistort: ", imgNotGood)
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

        # undistort
        mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
        dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        print("ROI: ", x, y, w, h)

        cv2.imwrite(workingFolder + "/calibresult.png",dst)
        print("Calibrated picture saved as calibresult.png")
        print("Calibration Matrix: ")
        print(mtx)
        print("Disortion: ", dist)
        #--------- Save result
        filename = workingFolder + "/camera_matrix.txt"
        np.savetxt(filename, mtx, delimiter=',')
        filename = workingFolder + "/camera_distortion.txt"
        np.savetxt(filename, dist, delimiter=',')

        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error

        print("total error: ", mean_error/len(objpoints))
    else:
        print("In order to calibrate you need at least 9 good pictures... try again")

