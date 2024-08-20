import numpy as np
import cv2 as cv
import glob
import os

print("script start")
# Display a blank image to create an OpenCV window
dummy_image = np.zeros((100, 100, 3), np.uint8)
cv.imshow("Dummy Image", dummy_image)
cv.waitKey(0)
# Get the directory of the script
script_dir = os.path.dirname(__file__)

# Join the script directory with the relative path
opencv_img_path = os.path.join(script_dir, "opencv_demo.jpg")
github_img_path = os.path.join(script_dir, "github_demo.jpg")
icuas_img_path = os.path.join(script_dir, "icuas_demo.png")
test_img_path = os.path.join(script_dir, "chessboard_test.png")
# opncv_image = cv.imread(opencv_img_path)
# print(f"script_dir: {script_dir}")
# print(f"img_dir: {opencv_img_path}")
img = cv.imread(test_img_path)
# Get the dimensions of the image
# height, width = img.shape[:2]
# Resize the image
# img = cv.resize(img, (width*2, height*2), interpolation = cv.INTER_CUBIC)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

chessboard_w = 9
chessboard_h = 6
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboard_w*chessboard_h,3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_w,0:chessboard_h].T.reshape(-1,2)
# print(f"objp: {objp}")
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
image_dir =os.path.join(script_dir, '/opencv_data')
print("the image dir is:")
print(image_dir)
print("the image dir is:")
image_files = glob.glob(os.path.join(image_dir, '*.jpg'))
 
images = glob.glob('./opencv_data/*.jpg') 
 

for fname in images:
 img = cv.imread(fname)
 # Get the original image dimensions
 original_height, original_width = img.shape[:2]
 # Calculate the new dimensions
 new_dimensions = (original_width * 2, original_height * 2)
 # Resize the image
 resized_image = cv.resize(img, new_dimensions, interpolation = cv.INTER_LINEAR)
 gray = cv.cvtColor(resized_image, cv.COLOR_BGR2GRAY)
 
 # Find the chess board corners
 ret, corners = cv.findChessboardCorners(gray, (chessboard_w,chessboard_h), None)
#  corners = corners/2
 # If found, add object points, image points (after refining them)
 if ret == True:
    objpoints.append(objp)
 
#  corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
 imgpoints.append(corners)
 
 # Draw and display the corners
#  cv.drawChessboardCorners(img, (chessboard_w,chessboard_h), corners, ret)
#  cv.imshow('img', img)

 cv.drawChessboardCorners(resized_image, (chessboard_w,chessboard_h), corners, ret)
 cv.imshow('img', resized_image)
 cv.waitKey(0)
 
cv.destroyAllWindows()

# calibration
this_img = cv.imread(script_dir+'/opencv_data/left12.jpg')
# this_gray = cv.cvtColor(this_img, cv.COLOR_BGR2GRAY)
if this_img is not None:
    this_gray = cv.cvtColor(this_img, cv.COLOR_BGR2GRAY)
else:
    print("Error loading image")

# print(f"Number of object points: {len(objpoints)}")
# print(f"Number of image points: {len(imgpoints)}")
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, this_gray.shape[::-1], None, None)
# print(f"mtx:\n {mtx}\n")

# img = cv.imread('left12.jpg')
h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
 
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
# iresult_path = os.path.join(script_dir, "./opencv_demo_result.jpg")
cv.imwrite('github_calibresult.png', dst)


mean_error = 0
for i in range(len(objpoints)):
 imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
 error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
 mean_error += error
 
print( "total error: {}".format(mean_error/len(objpoints)) )