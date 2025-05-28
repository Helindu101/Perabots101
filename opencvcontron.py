from controller import Robot, Camera, Motor
import numpy as np
import cv2
if name == "main":
# Create the Robot instance and define timestep
robot = Robot()
timestep = 64
max_speed = 6.28
# Initialize camera
cam = robot.getDevice('camera')
cam.enable(timestep)
width = cam.getWidth()
height = cam.getHeight()

# Initialize motors
left_motor = robot.getDevice('motor_1')
right_motor = robot.getDevice('motor_2')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Define HSV range for black detection
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 50])  # Adjust V max if needed

# Kernel for morphological operations
kernel = np.ones((3, 3), np.uint8)

while robot.step(timestep) != -1:
    # Capture and preprocess image
    image = cam.getImage()
    img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Create mask for black-colored obstacles
    obstacle_mask = cv2.inRange(img_hsv, lower_black, upper_black)

    # Clean up mask: remove noise and fill gaps
    obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
    obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_DILATE, kernel)

    # Find contours of detected black objects
    contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw detected contours on original image
    output_img = img_bgr.copy()
    if contours:
        cv2.drawContours(output_img, contours, -1, (0, 0, 255), 2)  # Red contours
        cv2.putText(output_img, "Black Obstacle Detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(output_img, "No Black Obstacle", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the result
    cv2.imshow("Robot View with Black Obstacle Overlay", output_img)
    cv2.waitKey(1)

    # Set robot motion (moves forward by default)
    left_motor.setVelocity(0.5 * max_speed)
    right_motor.setVelocity(0.5 * max_speed)

