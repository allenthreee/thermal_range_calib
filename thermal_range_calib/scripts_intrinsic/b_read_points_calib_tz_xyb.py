import numpy as np
import cv2 as cv
import glob
import os



def read_2d_points(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
    # Split each line into x and y coordinates and convert them to float
    points = [list(map(float, line.strip().split())) for line in lines]
    
    # Convert the list of points into a numpy array and reshape it to match the shape of 'corners'
    corners = np.array(points, dtype='float32').reshape(-1, 1, 2)
    
    return corners

# Get the directory of the script
script_dir = os.path.dirname(__file__)


# Join the script directory with the relative path
# opencv_img_path = os.path.join(script_dir, "opencv_data/left01.jpg")
github_img_path = os.path.join(script_dir, "github_demo.jpg")
icuas_img_path = os.path.join(script_dir, "icuas_demo.png")
six_four_chessboard = os.path.join(script_dir, "chessboard_test.png")
# opncv_image = cv.imread(opencv_img_path)
# print(f"script_dir: {script_dir}")
# print(f"img_dir: {opencv_img_path}")
# img = cv.imread(opencv_img_path)


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

chessboard_w = 6
chessboard_h = 6
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboard_w*chessboard_h,3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_w,0:chessboard_h].T.reshape(-1,2)
print(f"objp: {objp}")
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
images = glob.glob('/opencv_data/*.jpg') 
 


# Find the chess board corners
# ret, corners = cv.findChessboardCorners(gray, (chessboard_w,chessboard_h), None)
# print(f"corners: {corners}\n")
# If found, add object points, image points (after refining them)


# Loop over the range
# for i in range(1, 4):
for i in (1, 4):
    # Construct the file name
    # img = cv.imread(f"opencv_data/left0{i}.jpg")
    img = cv.imread(f"icuas_data/icuas_0{i}.png")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Read the corners from the file
    file_name = f"./icuas_0{i}_放大二倍.txt"
    corners = read_2d_points(file_name)
    corners = corners/2
    
    # Print the corners
    print(f"corners from {file_name}: {corners}\n")
    
    # Append the corners to the list
    imgpoints.append(corners)
    # Draw and display the corners
    cv.drawChessboardCorners(img, (chessboard_w,chessboard_h), corners, True)
    cv.imshow('img', img)
    if True == True:
        objpoints.append(objp)
    cv.waitKey(0)

# corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
# imgpoints.append(corners)


 

# print(f"imgpoints: {imgpoints}, \n\ncorners.shape() is {corners.shape}")
print(f"Number of object points: {len(objpoints)}")
print(f"Number of image points: {len(imgpoints)}")

 
cv.destroyAllWindows()

# calibration
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(f"mtx:\n {mtx}\n")

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