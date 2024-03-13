import cv2
import numpy as np

# Let's load a simple image with 3 black squares
image = cv2.imread("/home/allen/calib_ws/data_demo/0thermal.png")
# cv2.waitKey(0)
print("load image")

# Callback function for trackbar changes
def update_params(x):
    # Get the current trackbar values
    threshold1 = cv2.getTrackbarPos('Threshold 1', 'Canny')
    threshold2 = cv2.getTrackbarPos('Threshold 2', 'Canny')

    # Apply Canny edge detection
    edges = cv2.Canny(image, threshold1, threshold2)

    # Display the result
    cv2.imshow('Canny', edges)

# Load an image
# image = cv2.imread('image.jpg', 0)  # Read the image in grayscale

# Create a named window
cv2.namedWindow('Canny')

# Create trackbars
cv2.createTrackbar('Threshold 1', 'Canny', 0, 255, update_params)
cv2.createTrackbar('Threshold 2', 'Canny', 0, 255, update_params)

# Initialize trackbar positions
cv2.setTrackbarPos('Threshold 1', 'Canny', 50)
cv2.setTrackbarPos('Threshold 2', 'Canny', 150)

# Initialize Canny edge detection with default parameters
edges = cv2.Canny(image, 50, 150)

# Display the initial result
cv2.imshow('Canny', edges)

# Wait for key press
cv2.waitKey(0)

# Destroy all windows
cv2.destroyAllWindows()
