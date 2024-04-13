import cv2

# Initialize the camera (change the index based on your camera availability)
webcam = cv2.VideoCapture(1)  # Use VideoCapture(0) or (1) as needed

# Capture a frame from the camera
_, frame = webcam.read()

# Release the camera
webcam.release()

# Specify the directory and filename for saving the image
image_filename = r'Resources/saved_img.jpg'

# Save the captured frame as a JPEG image
cv2.imwrite(filename=image_filename, img=frame)