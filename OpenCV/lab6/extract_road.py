import cv2
import numpy as np

# Load the image
image = cv2.imread('/home/achanti/shot.png')

# Check if image is loaded properly
if image is None:
    print("Error: Image not found!")
    exit()

# Convert the image to HSV format for correct color extraction
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Mouse callback function to print HSV values on click
def mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        h = hsv_image[y, x, 0]
        s = hsv_image[y, x, 1]
        v = hsv_image[y, x, 2]
        print("H:", h)
        print("S:", s)
        print("V:", v)

# Create a window and set the mouse callback for HSV extraction
cv2.namedWindow('image')
cv2.setMouseCallback('image', mouse)

# Show the original image
cv2.imshow('image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Define HSV range for line detection (modify the values as needed)
light_line = np.array([0, 150, 0])  # Example: lower bound of HSV
dark_line = np.array([180, 255, 255])  # Example: upper bound of HSV

# Create a mask using the defined HSV range
mask = cv2.inRange(hsv_image, light_line, dark_line)
cv2.imshow('mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Apply Canny edge detection
canny = cv2.Canny(mask, 50, 150)
cv2.imshow('edge', canny)
cv2.waitKey(0)
cv2.destroyAllWindows()
print(canny.shape)

# Crop the Canny edge-detected image (ensure valid crop dimensions)
r1 = 200
c1 = 0
img = canny[r1:r1+200, c1:c1+512]
cv2.imshow('crop', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Find edges on the cropped image
edge = []
row = 150
for i in range(512):
    if img[row, i] == 255:
        edge.append(i)
print(edge)

# Initialize left and right edges based on the number of edges found
left_edge = None
right_edge = None

if len(edge) == 4:
    left_edge = edge[0]
    right_edge = edge[2]
elif len(edge) == 3:
    if edge[1] - edge[0] > 5: 
        left_edge = edge[0]
        right_edge = edge[1]
    else:
        left_edge = edge[0]
        right_edge = edge[2]

# Calculate road width and frame mid-point if edges are found
if left_edge is not None and right_edge is not None:
    road_width = (right_edge - left_edge)
    frame_mid = left_edge + (road_width / 2)
    mid_point = 512 / 2

    # Mark the mid-point of the frame and calculate error
    img[row, int(mid_point)] = 255
    print("Mid-point of the frame:", mid_point)
    error = mid_point - frame_mid

    # Determine action based on error
    action = "Go Right" if error < 0 else "Go Left"
    print("Error:", error)

    # Mark the calculated frame mid-point and display the action text
    img[row, int(frame_mid)] = 255
    f_image = cv2.putText(img, action, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
    cv2.imshow('final image', f_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Error: Could not detect sufficient edges.")

