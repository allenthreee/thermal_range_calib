import cv2

# Initialize global variables
start_point = None
end_point = None

# Mouse callback function
def draw_line(event, x, y, flags, param):
    global start_point, end_point

    if event == cv2.EVENT_LBUTTONDOWN:
        start_point = (x, y)
        cv2.putText(image, f"({x}, {y})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        print(f"Start point: {start_point}")
    elif event == cv2.EVENT_RBUTTONDOWN:
        end_point = (x, y)
        cv2.putText(image, f"({x}, {y})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        print(f"End point: {end_point}")
        if start_point is not None:
            cv2.line(image, start_point, end_point, (0, 255, 0), 2)
            cv2.imshow("Image", image)

# Load an image
# /calib_data/ign_simulation/ldiar80.5_thermal810.5
image_path = "/home/allen/calib_data/ign_simulation/ldiar80.5_thermal810.5/640x480.png"
image = cv2.imread(image_path)

# Create a window and set the mouse callback
cv2.namedWindow("Image")
cv2.setMouseCallback("Image", draw_line)

# Display the image and wait for a key press
cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
