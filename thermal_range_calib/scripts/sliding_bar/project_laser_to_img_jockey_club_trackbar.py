import numpy as np
import cv2
import open3d as o3d

# Load point cloud data from PCD file
def load_point_cloud(filename):
    pcd = o3d.io.read_point_cloud(filename)
    return np.asarray(pcd.points)

# Define the camera matrix
fx = 607.4325
fy = 606.4396
cx = 317.5857
cy = 243.9464
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], np.float32)

# Define the distortion coefficients
dist_coeffs = np.zeros((5, 1), np.float32)

# Load the point cloud
pcd_arr = load_point_cloud("/home/allen/calib_data/cjy_single_scene_calibration/jokey_club/jokey_club_downsampled.pcd")
# Load the image
image = cv2.imread("/home/allen/calib_data/cjy_single_scene_calibration/jokey_club/rgb.png")

# Initialize rotation and translation vectors
rvec = np.array([0.5773503*2.0943951, -0.5773503*2.0943951, 0.5773503*2.0943951], np.float32)
tvec = np.zeros((3, 1), np.float32)

# Create a named window
cv2.namedWindow('Projection')

# Define trackbar callback function
def update_params(x):
    global image  # Declare 'image' as a global variable
    global pcd_arr  # Declare 'image' as a global variable
    # Get the current trackbar values
    # rvec_euler = np.array([cv2.getTrackbarPos('Rotation X', 'Projection') / 2000.0,
    #                       cv2.getTrackbarPos('Rotation Y', 'Projection') / 2000.0,
    #                       cv2.getTrackbarPos('Rotation Z', 'Projection') / 2000.0], np.float32)

    # rvec, _ = cv2.Rodrigues(rvec_euler.astype(np.float64))
    # tvec[0] = cv2.getTrackbarPos('Translation X', 'Projection') / 2000.0
    # tvec[1] = cv2.getTrackbarPos('Translation Y', 'Projection') / 2000.0
    # tvec[2] = cv2.getTrackbarPos('Translation Z', 'Projection') / 2000.0
	
    rvec[0] = cv2.getTrackbarPos('Rotation X /1000', 'Projection') / 1000.0
    rvec[1] = cv2.getTrackbarPos('Rotation Y /-1000', 'Projection') / -1000.0
    rvec[2] = cv2.getTrackbarPos('Rotation Z /1000', 'Projection') / 1000.0
    tvec[0] = cv2.getTrackbarPos('Translation X /1000', 'Projection') / 1000.0
    tvec[1] = cv2.getTrackbarPos('Translation Y /-1000', 'Projection') / 1000.0 
    tvec[2] = cv2.getTrackbarPos('Translation Z /-1000', 'Projection') / 1000.0 
    # tvec[0] = tvec[0] - 200
    # tvec[1] = tvec[1] - 200
    # tvec[2] = tvec[2] - 200
    # Map the 3D point to 2D point
    points_2d, _ = cv2.projectPoints(pcd_arr,
                                     rvec, tvec,
                                     camera_matrix,
                                     dist_coeffs)

    # Create a copy of the original image
    resized_image = np.copy(image)

    # Draw projected points on the resized image
    for i in range(len(pcd_arr)):
        if(i%2==0):
            continue
        if(i%3==0):
            continue
        if(i%5==0):
            continue
        dist = cv2.norm(pcd_arr[i])
        if(cv2.norm(points_2d[i][0])>300000):
            continue
        if(pcd_arr[i][0]<0):
            continue
        dist = cv2.norm(pcd_arr[i])
        # print(dist)
        R = 255+(0-255)*int(abs(dist))/59.01
        G = (0+(255-0)*int(abs(dist))/59.01)*0.5
        B = 0+(255-0)*int(abs(dist))/59.01
        resized_image = cv2.circle(resized_image, (int(points_2d[i][0][0]), int(points_2d[i][0][1])), radius=0, color=(B, G, R), thickness=5)
        # print("cv2.cirle called")
        
    # Resize the image to 1280x720
    resized_image = cv2.resize(resized_image, (1280, 720))

    # Display the result
    cv2.imshow('Projection', resized_image)

# Create trackbars for rotation and translation
cv2.createTrackbar('Rotation X /1000', 'Projection', 0, 2000, update_params)
cv2.createTrackbar('Rotation Y /-1000', 'Projection', 0, 2000, update_params)
cv2.createTrackbar('Rotation Z /1000', 'Projection', 0, 2000, update_params)
cv2.createTrackbar('Translation X /1000', 'Projection', 0, 2000, update_params)
cv2.createTrackbar('Translation Y /-1000', 'Projection', 0, 2000, update_params)
cv2.createTrackbar('Translation Z /-1000', 'Projection', 0, 2000, update_params)
# Set initial values for rotation
cv2.setTrackbarPos('Rotation X /1000', 'Projection', int(0.5773503*2.0943951*1000))
cv2.setTrackbarPos('Rotation Y /-1000', 'Projection', int(0.5773503*2.0943951*1000))
cv2.setTrackbarPos('Rotation Z /1000', 'Projection', int(0.5773503*2.0943951*1000))
# Call the update_params function to display the initial projection
update_params(0)

# Wait for key press
cv2.waitKey(0)

# Destroy all windows
cv2.destroyAllWindows()