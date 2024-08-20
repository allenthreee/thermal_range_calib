import numpy as np
import glob
import os
import cv2 as cv
import numpy as np
import pickle

# Get the directory of the script
script_dir = os.path.dirname(__file__)

# Join the script directory with the relative path
# opencv_img_path = os.path.join(script_dir, "./opencv_data/left03.jpg")
# save_picked_points_dir = "./opencv_left03.txt"
# image = cv.imread(opencv_img_path)

icuas_img_path = os.path.join(script_dir, "icuas_data/icuas_04.png")
save_picked_points_dir = "./icuas_04_放大二倍.txt"
image = cv.imread(icuas_img_path)

# Initialize the list of points.
points = []

chessboard_w = 6
chessboard_h = 6

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboard_w*chessboard_h,3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_w,0:chessboard_h].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

def draw_circle(image, center):
    cv.point(image, center, 1, (0, 0, 255), thickness = 1)

def draw_point(image, point):
    cv.rectangle(image, point, point, (0, 0, 255), 1)
    # cv.line(image, (center[0] - 1, center[1] - 1), (center[0] + 1, center[1] + 1), (0, 0, 255), thickness=1)
    # cv.line(image, (center[0] - 1, center[1] + 1), (center[0] + 1, center[1] - 1), (0, 0, 255), thickness=1)

def select_points(event, x, y, flags, param):
    global points, image_copy
    # If the left mouse button was clicked, record the (x, y) coordinates.
    if event == cv.EVENT_LBUTTONDOWN:
        points.append((x, y))
        # draw_circle(image_copy, (x, y))
        draw_point(image_copy, (x, y))

    # If the right mouse button was clicked, end the selection process.
    elif event == cv.EVENT_RBUTTONDOWN:
        cv.destroyAllWindows()
        # Save the points to a file
        with open('points.pkl', 'wb') as f:
            pickle.dump(points, f)
        # Save the image
        cv.imwrite('marked_image.jpg', image_copy)

# Load your image.
# image = cv.imread('your_image.jpg')

# Create a copy of the image to draw on.
# Get the original image dimensions
original_height, original_width = image.shape[:2]
# Calculate the new dimensions
new_dimensions = (original_width * 2, original_height * 2)
# Resize the image
resized_image = cv.resize(image, new_dimensions, interpolation = cv.INTER_LINEAR)

image_copy = resized_image.copy()

# Set up the mouse event callback.
cv.namedWindow('image')
cv.setMouseCallback('image', select_points)

# Keep looping until the 'q' key is pressed or right mouse button is clicked.
while True:
    # Display the image.
    cv.imshow('image', image_copy)

    # If 'q' is pressed, break from the loop.
    if cv.waitKey(1) & 0xFF == ord('q'):
        np.savetxt(save_picked_points_dir, points, fmt='%d')
        break

# Convert the list to the required format
corners = np.array(points, dtype=np.float32).reshape(-1, 1, 2)

objpoints.append(objp)
imgpoints.append(corners)
print(f"obj_points: {objpoints}")
print(f"corners: {corners}, \n\ncorners.shape() is {corners.shape}")

# Draw and display the corners
cv.drawChessboardCorners(resized_image, (chessboard_w,chessboard_h), corners, True)
cv.imshow('visual_check_img', resized_image)
cv.waitKey(0)

# calibration
gray = cv.cvtColor(resized_image, cv.COLOR_BGR2GRAY)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[1::-1], None, None)
print(f"mtx:\n {mtx}\n")
# h, w = image.shape[:2]
# newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# Close all windows.
cv.destroyAllWindows()

# Print the selected points.
print("Selected points:", points)
