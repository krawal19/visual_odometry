import cv2
import glob
import numpy as np
from ReadCameraModel import*
from UndistortImage import*

images = glob.glob('Oxford_dataset\stereo\centre\*.png')

# Preparing the data , GraytoColor conversion, Extract camera parameters, Undistorted image
def dataPrep(imgs):
    # Convert the Bayer pattern encoded imageimgto acolorimage
    color_image = cv2.cvtColor(imgs,cv2.COLOR_BayerGR2BGR)
    # Extract the camera parameters usingReadCameraModel.py
    fx, fy, cx, cy, G_camera_image, LUT = ReadCameraModel('Oxford_dataset\model')
    undistorted_image = UndistortImage(color_image,LUT)
    return undistorted_image

for img in images:
    imgs = cv2.imread(img,0)
    undistortImg  = dataPrep(imgs) 
    cv2.imwrite("undistorted/" + img, undistortImg)

