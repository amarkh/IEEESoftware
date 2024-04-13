## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import copy
import math

def reload():
    retval = {}
    with open("config", "r") as f:
        for line in f.readlines():
            d = eval('{\"' + line.split(":")[0] + '\" : ' + line.split(":")[1] + "}")
            retval.update(d)
    print(retval)
    return retval

# Function to convert 2D image coordinates to 3D world coordinates
def get_3d_coordinates(x, y, depth_value, depth_intrin):
    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth_value)
    return depth_point

def make3d(img):
    return np.dstack([img[:,:, None]] * 3)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 60) # 640, 80, 30

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 60) # 960, 540, 30
else:
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 60) # 640, 480, 30

# Start streaming
pipeline.start(config)
config = reload()
locals().update(config)

print("")
print(thresh_lower)
print(thresh_upper)
print(kernel)
bruh = 0

try:
# while bruh == 0:
    rows = []
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    # if not color_frame:
    #     continue

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    
    masked_image = cv2.inRange(hsv_image, thresh_lower, thresh_upper)

    

    img_eroded = cv2.erode(masked_image, kernel, iterations=1)
    img_dilated = cv2.dilate(img_eroded, kernel, iterations=dilations)

    # skeleton
    # opencvpython.blogspot.com/2012/05/skeletonization-using-opencv-python.html
    elt = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
    elt = np.ones((3, 3), np.uint8)
    skel_img = img_dilated.copy()
    size = np.size(img_dilated)
    
    skel = np.zeros(img_dilated.shape, np.uint8)
    while True:
        eroded = cv2.erode(skel_img, elt)
        temp = cv2.dilate(eroded, elt)
        temp = cv2.subtract(skel_img, temp)
        skel = cv2.bitwise_or(skel, temp)
        skel_img = eroded.copy()


        zeros = size - cv2.countNonZero(skel_img)
        if zeros == size:
            break

    eroded_skel = cv2.erode(skel, skel_kernel)
    lines = cv2.HoughLines(skel, 1, np.pi / 180, threshold, None, 0, 0)
    # lines = cv2.HoughLinesP(skel, )

    

    img_with_lines = copy.deepcopy(color_image)
    if lines is not None:
        rho = lines[0][0][0]
        theta = lines[0][0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + color_image.shape[1]*(-b)), int(y0 + color_image.shape[0]*(a)))
        pt2 = (int(x0 - color_image.shape[1]*(-b)), int(y0 - color_image.shape[0]*(a)))
        cv2.line(img_with_lines, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

    test_lines = color_image.copy()
    # cv2.line(test_lines, (int(test_lines.shape[0] / 2), 0), (int(test_lines.shape[0] / 2), int(test_lines.shape[1])), (0,0,255), cv2.LINE_AA)
    # cv2.line(test_lines, (int(test_lines.shape[1] / 2), 0), (int(test_lines.shape[1] / 2), int(test_lines.shape[0])), (0,255,0), cv2.LINE_AA)
    # cv2.line(test_lines, (0, int(test_lines.shape[0] / 2)), (int(test_lines.shape[1]), int(test_lines.shape[0] / 2)), (255,0,0), cv2.LINE_AA)
    # cv2.line(test_lines, (0, int(test_lines.shape[1] / 2)), (int(test_lines.shape[0]), int(test_lines.shape[1] / 2)), (255,0,255), cv2.LINE_AA)
    # cv2.line(test_lines, (30, 100), (100, 300), (255,255,0), cv2.LINE_AA)

#This Sums the lines and works
    # if lines is not None:
    #     sum_lines = [0, 0]
    #     for line in lines:
    #         sum_lines[0] += line[0][0]
    #         sum_lines[1] += line[0][1]
    #     avg_line = [l / len(lines) for l in sum_lines]
    #     rho = avg_line[0]
    #     theta = avg_line[1]
    #     a = math.cos(theta)
    #     b = math.sin(theta)
    #     x0 = a * rho
    #     y0 = b * rho
    #     pt1 = (int(x0 + color_image.shape[1]*(-b)), int(y0 + color_image.shape[0]*(a)))
    #     pt2 = (int(x0 - color_image.shape[1]*(-b)), int(y0 - color_image.shape[0]*(a)))
    #     cv2.line(img_with_lines, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

    # if lines is not None:
    #     for line in lines:
    #         x1, y1, x2, y2 = line[0][0], line[0][1], line[0][0], line[1][1]
    #         cv2.line(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    masked_3d = make3d(masked_image)
    eroded_3d = make3d(img_eroded)
    dilated_3d = make3d(img_dilated)
    skel_3d = make3d(skel)
    eroded_skel_3d = make3d(eroded_skel)

    rows.append(np.hstack((test_lines, skel_3d)))
    rows.append(np.hstack((dilated_3d, img_with_lines)))
    #[print(i.shape) for i in rows]

    # rows.append(np.hstack((masked_3d, np.zeros(masked_3d.shape))))
    images = np.vstack(rows)

    # Get depth intrinsics for 3D coordinate conversion
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

    allX = []
    allY = []
    allZ = []

    # Iterate over the pixels of the detected line in eroded_skel
    for y in range(eroded_skel.shape[0]):
        for x in range(eroded_skel.shape[1]):
            if eroded_skel[y, x] == 255:  # Assuming white pixels represent the line
                depth_value = depth_frame.get_distance(x, y)
                if depth_value > 0:
                    # Convert 2D pixel coordinates to 3D world coordinates
                    coordinates_3d = get_3d_coordinates(x, y, depth_value, depth_intrin)
                    print("3D Coordinates:", coordinates_3d)
                    allX.append(coordinates_3d[0])
                    allY.append(coordinates_3d[1]) # Up and down
                    allZ.append(coordinates_3d[2]) # Forward and back

                    

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    key = cv2.waitKey(1)

    # process comands
    
    # if key == 114: # r
    #     config = reload()
    #     locals().update(config)
    #     print("")
    #     print(thresh_lower)
    #     print(thresh_upper)
    #     print(kernel)
    #     continue
    # # if (key == 108) or (key == 113):
    # #     lines = cv2.HoughLines(img_eroded, 1, np.pi / 180, threshold, None, 0, 0)
    # #     linesP = cv2.HoughLinesP(img_eroded, 1, np.pi/180, threshold, None, 0, 0)
    # if key == 108: # l
    #     print(lines)
    #     print(type(lines))
    #     try:
    #         print(lines.shape)
    #     except AttributeError as e:
    #         pass
    # if key == 113:# q
    #     break
    # if key == 115: # s
    #     print(color_image.shape)
    #     continue
    # if key != -1:
    #     print(key)

    while True:
        x = 1

finally:

    # Stop streaming
    pipeline.stop()