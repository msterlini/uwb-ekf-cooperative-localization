# Some pieces of code are taken from github of
# Author: Tiziano Fiorenzani

# This code will run as a node that continuously processes the image stream coming from the robot's camera and will identify yellow spheres (the target) when present in the image.
# It will also publish the coordinate of the center of the recognized shape.
# It will make heavy use of the cv2 python module.

import rospy
import cv2
import numpy as np

# ROS specific imports
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class BlobDetector:

    # Initializer definition
    def __init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None):

        # Parameters setting
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window

        # Variable holding the point
        self.blob_point = Point()

        # Publishing topics
        self.new_image_pub = rospy.Publisher("target/image_blob", Image, queue_size=1)
        self.blob_pub = rospy.Publisher("target/point_blob", Point, queue_size=1)

        # Initialize CV Bridge to convert ROS Image to OpenCV image format
        self.bridge = CvBridge()

        # Subscribing topics
        self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)

    # Blob detecting function: returns keypoints and mask
    def blob_detect(self, image,                  # The frame (cv standard)
                    hsv_min,                # minimum threshold of the hsv filter [h_min, s_min, v_min]
                    hsv_max,                # maximum threshold of the hsv filter [h_max, s_max, v_max]
                    blur=0,                 # blur value (default 0)
                    blob_params=None,       # blob parameters (default None)
                    search_window=None      # window where to search as [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
                ):

        # Blur image to remove noise
        if blur > 0: 
            image = cv2.blur(image, (blur, blur))

        # Search window 
        # Smaller than whole window to scan only for objects in front of the unicycle
        if search_window is None: 
            search_window = [0.0, 0.0, 1.0, 1.0]

        # Convert image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Apply HSV threshold
        mask = cv2.inRange(hsv, hsv_min, hsv_max)

        # Dilate makes the in range areas larger
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.erode(mask, None, iterations=2)

        # Cut the image using the search mask
        mask = self.apply_search_window(mask, search_window)

        # Build default blob detection parameters, if none have been provided
        if blob_params is None:
            # Set up the SimpleBlobDetector of the cv2 module with default parameters.
            params = cv2.SimpleBlobDetector_Params()
            
            # Change thresholds
            params.minThreshold = 0
            params.maxThreshold = 100
            
            # Filter by Area.
            params.filterByArea = True
            params.minArea = 30
            params.maxArea = 20000
            
            # Filter by Circularity
            params.filterByCircularity = True
            params.minCircularity = 0.1
            
            # Filter by Convexity
            params.filterByConvexity = True
            params.minConvexity = 0.5
            
            # Filter by Inertia
            params.filterByInertia = True
            params.minInertiaRatio = 0.5
        else:
            params = blob_params     

        # Apply blob detection
        detector = cv2.SimpleBlobDetector_create(params)

        # Reverse the mask: blobs are black on white
        reversemask = 255 - mask

        # Keypoints extraction
        keypoints = detector.detect(reversemask)

        return keypoints, reversemask

    # Draw detected blobs: returns the image
    def draw_keypoints(self, image,                   # Input image
                       keypoints,               # CV keypoints
                       line_color=(0,0,255)     # line's color (b,g,r)
                    ):
        
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
        return im_with_keypoints

    # Draw search window: returns the image
    def draw_window(self, image,              # Input image
                    window_adim,        # window in adimensional units
                    color=(255,0,0),    # line's color
                    line=5              # line's thickness
                ):
        
        rows = image.shape[0]
        cols = image.shape[1]
        
        x_min_px = int(cols * window_adim[0])
        y_min_px = int(rows * window_adim[1])
        x_max_px = int(cols * window_adim[2])
        y_max_px = int(rows * window_adim[3])  
        
        # Draw a rectangle from top left to bottom right corner
        image = cv2.rectangle(image, (x_min_px, y_min_px), (x_max_px, y_max_px), color, line)

        return image

    # Draw X Y frame
    def draw_frame(self, image,
                   dimension=0.3,      # dimension relative to frame size
                   line=2              # line's thickness
                ):
        
        rows = image.shape[0]
        cols = image.shape[1]
        size = min([rows, cols])
        center_x = int(cols / 2.0)
        center_y = int(rows / 2.0)
        
        line_length = int(size * dimension)
        
        # Draws 2 lines in order to make the reference frame 
        # X
        image = cv2.line(image, (center_x, center_y), (center_x + line_length, center_y), (0,0,255), line)
        # Y
        image = cv2.line(image, (center_x, center_y), (center_x, center_y + line_length), (0,255,0), line)
        
        return image

    # Apply search window: returns the image
    def apply_search_window(self, image, window_adim=[0.0, 0.0, 1.0, 1.0]):
        rows = image.shape[0]
        cols = image.shape[1]
        x_min_px = int(cols * window_adim[0])
        y_min_px = int(rows * window_adim[1])
        x_max_px = int(cols * window_adim[2])
        y_max_px = int(rows * window_adim[3])    
        
        # Initialize the mask as a black image
        mask = np.zeros(image.shape, np.uint8)
        
        # Copy the pixels from the original image corresponding to the window
        mask[y_min_px:y_max_px, x_min_px:x_max_px] = image[y_min_px:y_max_px, x_min_px:x_max_px]   
        
        # return the mask
        return mask
        
    # Apply a blur to the outside search region
    def blur_outside(self, image, blur=5, window_adim=[0.0, 0.0, 1.0, 1.0]):
        rows = image.shape[0]
        cols = image.shape[1]
        x_min_px = int(cols * window_adim[0])
        y_min_px = int(rows * window_adim[1])
        x_max_px = int(cols * window_adim[2])
        y_max_px = int(rows * window_adim[3])    
        
        # Initialize the mask as a black image
        mask = cv2.blur(image, (blur, blur))
        
        # Copy the pixels from the original image corresponding to the window
        mask[y_min_px:y_max_px, x_min_px:x_max_px] = image[y_min_px:y_max_px, x_min_px:x_max_px]   
        
        # return the mask
        return mask
        
    # Obtain the camera relative frame coordinate of one single keypoint
    def get_blob_relative_position(self, image, keyPoint):
        rows = float(image.shape[0])
        cols = float(image.shape[1])
        center_x = 0.5 * cols
        center_y = 0.5 * rows
        x = (keyPoint.pt[0] - center_x) / center_x
        y = (keyPoint.pt[1] - center_y) / center_y
        return x, y

    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]

    def set_blur(self, blur):
        self._blur = blur

    def set_blob_params(self, blob_params):
        self._blob_params = blob_params

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            # Detect blobs
            keypoints, mask = self.blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                               blob_params=self._blob_params, search_window=self.detection_window)
            # Draw search window and blobs
            cv_image = self.blur_outside(cv_image, 10, self.detection_window)
            cv_image = self.draw_window(cv_image, self.detection_window, line=1)
            cv_image = self.draw_frame(cv_image)
            cv_image = self.draw_keypoints(cv_image, keypoints)

            try:
                self.new_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

            for i, keyPoint in enumerate(keypoints):
                # Here you can implement some tracking algorithm to filter multiple detections
                # We are simply getting the first result
                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size
                print("kp %d: s = %3d   x = %3d  y= %3d" % (i, s, x, y))
                
                # Find x and y position in camera adimensional frame
                x, y = self.get_blob_relative_position(cv_image, keyPoint)
                
                self.blob_point.x = x
                self.blob_point.y = y
                self.blob_pub.publish(self.blob_point)
                break

if __name__ == '__main__':

    # HSV mask parameters to filter yellow color
    yellow_min = (20, 100, 100)
    yellow_max = (30, 255, 255)

    # Blur intensity
    blur = 5

    # Minimum and maximum size of blob to detect 
    min_size = 10
    max_size = 40

    # Detection windows dimensions
    x_min = 0.2
    x_max = 0.8
    y_min = 0.1
    y_max = 0.5

    detection_window = [x_min, y_min, x_max, y_max]

    params = cv2.SimpleBlobDetector_Params()

    # List of parameters to pass to CV function BlobDetector
    
    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 100
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 20000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.7   
        
    # Node initialization
    rospy.init_node('blob_detector', anonymous=True)

    # Blob detector class initialization
    ic = BlobDetector(yellow_min, yellow_max, blur, params, detection_window)

    try:
        rospy.spin()
        # We don't need to cast any particular function since the detector already works on the callbacks of the topics
    except rospy.ROSInternalException:
        rospy.loginfo("Blob detector node interrupted")