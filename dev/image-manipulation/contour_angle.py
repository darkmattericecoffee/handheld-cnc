import cv2
import numpy as np
import matplotlib.pyplot as plt

def calculate_angle(pt1, pt2,offset):
    """Calculate the angle between two points."""
    dx = pt2[0] - pt1[0]
    dy = pt2[1] - pt1[1]
    angle = np.degrees(np.arctan2(dy, dx)) - offset
    angle = angle % 360
    if angle > 180:
        angle = 360 - angle
    return angle

def rotate_point(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + np.cos(angle) * (px - ox) - np.sin(angle) * (py - oy)
    qy = oy + np.sin(angle) * (px - ox) + np.cos(angle) * (py - oy)
    return qx, qy

def show_image(title, image):
    """Helper function to display an image with a title."""
    plt.figure(figsize=(5, 5))
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title(title)
    plt.axis('off')
    plt.show()

def show_image_with_axis(title, image, orientation, save_path=None):
    """Helper function to display an image with a title and custom axis at the bottom."""
    plt.figure(figsize=(6, 6))
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    # plt.title(title)
    plt.axis('off')
    
    # Convert orientation to radians
    orientation_rad = np.radians(orientation)
    
    # Center point for the axis to start
    center_x = image.shape[1] // 2
    center_y = image.shape[0] - 200  # A little above the bottom
    
    # Length of the arrows
    arrow_length = 200
    
    # Calculate the end points of the arrows after rotation
    x_arrow_end = rotate_point((center_x, center_y), 
                               (center_x + arrow_length, center_y), 
                               orientation_rad)
    y_arrow_end = rotate_point((center_x, center_y), 
                               (center_x, center_y - arrow_length), 
                               orientation_rad)
    
    # Arrow properties
    arrow_props = dict(facecolor='white', edgecolor='white', arrowstyle='-|>', linewidth=1)
    
    # Drawing the arrows
    plt.gca().annotate('', xy=x_arrow_end, xytext=(center_x, center_y), arrowprops=arrow_props)
    plt.gca().annotate('', xy=y_arrow_end, xytext=(center_x, center_y), arrowprops=arrow_props)

    dot_size = 5  # Size of the dots
    plt.scatter([center_x, center_x], [center_y, center_y], color='white', s=dot_size)
    
    # Adding labels for x and y
    #plt.text(x_arrow_end[0]+10, x_arrow_end[1]+10, 'x', color='white', fontsize=12, ha='center', va='center')
    #plt.text(y_arrow_end[0]+10, y_arrow_end[1]+10, 'y', color='white', fontsize=12, ha='center', va='center')

    # Save the figure if a save path is provided
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300, pad_inches=0)  # bbox_inches='tight' removes excess whitespace
    
    plt.show()

# Load the image
img_path = 'data/cal.png'  # Replace with your image path
img = cv2.imread(img_path)

# Image manipulation
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # Convert to grayscale and apply GaussianBlur
blur = cv2.GaussianBlur(gray, (5, 5), 0)
# edges = cv2.Canny(blur, 100, 200) # Use Canny edge detection
thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
thresh_inv = cv2.bitwise_not(thresh)

# Find contours
contours, _ = cv2.findContours(thresh_inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(len(contours))
# main_contour = max(contours, key=cv2.contourArea)

# Create an image to draw the line segments
# img_line_segments = np.zeros_like(img)            # initiate as black background
extended_height = img.shape[0] + 400
img_line_segments = np.zeros((extended_height, img.shape[1], 3), dtype=np.uint8)


# Properties
orientation = 0

for contour in contours:
    # Approximate the contour to a polygon
    ep_val = 0.0001                 # Adjust this value to get more or fewer segments
    epsilon = ep_val * cv2.arcLength(contour, True)  
    approx_poly = cv2.approxPolyDP(contour, epsilon, True)

    # Draw each segment with the coloring logic based on its angle
    red_angle = 25
    yellow_angle = 45
    for i in range(len(approx_poly) - 1):
        pt1 = tuple(approx_poly[i][0])
        pt2 = tuple(approx_poly[i + 1][0])

        # Calculate the angle of the line segment
        angle = calculate_angle(pt1, pt2, orientation)

        # Choose color based on the angle
        if (0 <= angle < red_angle) or (180 - red_angle < angle <= 180):
            color = (0, 0, 255)  # Red
        elif (angle < yellow_angle)  or (angle > 180 - yellow_angle):
            color = (0, 255, 255)  # Yellow
        else:
            color = (0, 255, 0)  # Green
            print(angle)

        # Draw the line segment
        cv2.line(img_line_segments, pt1, pt2, color, 2)

# Save the output
output_path = f'output/contoured_logo_line_segments_{orientation}.png'  # Replace with your desired output path
cv2.imwrite(output_path, img_line_segments)

# Display the image
output_path_fig = f'output/contoured_fig_line_segments_{orientation}.png'
show_image_with_axis("Test", img_line_segments, orientation, output_path_fig)
# plt.imshow(cv2.cvtColor(img_line_segments, cv2.COLOR_BGR2RGB))
# plt.axis('off')
# plt.show()
