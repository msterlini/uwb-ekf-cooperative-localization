# Scripts

These python scripts hold all the nodes and logic of the project

## `gazebo_init.py`
## Key Features
This node dynamically spawns tags, targets, robots and the room model in the Gazebo simulation environment. It also recursively initialize all the other ROS nodes for each robot based on the specified number of lines in the csv file.
It also continuosly publishes static transforms for the tags, aiding in localization tasks.

## Topics
The script interacts with the following topics:
- `/gazebo/spawn_urdf_model`: Service used to spawn URDF models in Gazebo.
- `/gazebo/spawn_sdf_model`: Service used to spawn SDF models in Gazebo.
- Static transforms are published for tags to the `/tf_static` topic using `tf2_ros.StaticTransformBroadcaster`.

## Parameters
The script imports various parameters from the `init.launch` file:
- **Tag Parameters**:
  - `tag_height`: Height at which to spawn tags on the ceiling.
  - `tag_distance_rate`: Frequency at which to publish tag positions.
  - `tag_coordinate_file`: CSV file containing coordinates for tags.
  - `tag_urdf_file`: URDF file for tag models.
- **Target Parameters**:
  - `target_coordinate_file`: CSV file containing coordinates for targets.
  - `target_sdf_file`: SDF file for target models.
- **Room Parameters**:
  - `room_sdf_file`: SDF file for the room model.
- **Robot Parameters**:
  - `robot_coordinate_file`: CSV file containing initial coordinates for robots.
  - `robot_description`: URDF description of the robot.
- **General Parameters**:
  - `initialization_time`: Time to wait before starting the initialization process.

## Functions
- **`spawn_tag(x, y, id)`**: Spawns a tag at the given coordinates using a URDF model.
- **`spawn_target(x, y, id)`**: Spawns a target at the given coordinates using an SDF model.
- **`spawn_robot(x, y, id)`**: Spawns a robot at the given coordinates using a URDF model.
- **`spawn_room()`**: Spawns the room model in the simulation using an SDF file.
- **`publish_static_transform(x, y, id)`**: Publishes the static transform for a tag.

## `uwb_dist_sim.py`

## Key Features
It dynamically retrieves and stores the positions of UWB anchors in the simulation environment, ti also computes the distance between the robot and each UWB anchor, adding realistic noise to the measurements, finally it publishes the calculated distances and corresponding anchor IDs to a ROS topic.

## Topics
The script interacts with the following topics:
- **Subscription**: 
  - `/ground_truth/state`: Subscribes to this topic to get the robot's current ground truth position.
- **Publication**:
  - `uwb_data_topic`: Publishes UWB distance data using a custom message type `uwb_data`.

## Functions
- **`get_anchor_pos()`**: Retrieves and stores the positions of UWB anchors using the `tf` package.
- **`subscribe_data(data)`**: Callback function for the ground truth subscriber, updates the robot's position, and triggers distance calculation.
- **`uwb_sim()`**: Calculates distances from the robot to each UWB anchor and calls the publish function.
- **`calculate_distance(uwb_pose)`**: Computes the distance between the robot and a given UWB anchor position, adding noise to simulate measurement errors.
- **`publish_data(ids, distances)`**: Publishes the calculated distances and anchor IDs to the `uwb_data_topic`.


# blob_detector.py

## Overview
`blob_detector.py` is a ROS (Robot Operating System) script that continuously processes image streams from a robot's camera to identify yellow spheres (targets) and publishes their coordinates. It utilizes the `cv2` (OpenCV) module extensively for image processing tasks.

## Key Features
- **Blob Detection**: Identifies yellow spheres in the image stream using color thresholding and blob detection.
- **Coordinate Publishing**: Publishes the coordinates of detected blobs to a ROS topic.
- **Visualization**: Visualizes detected blobs and the search window on the processed image.

## Topics
The script interacts with the following topics:
- **Subscription**:
  - `camera/image_raw`: Subscribes to the raw image stream from the robot's camera.
- **Publication**:
  - `target/image_blob`: Publishes the processed image with detected blobs.
  - `target/point_blob`: Publishes the coordinates of the detected blob.

## Parameters
The script allows for several configurable parameters, such as:
- **Thresholds**: HSV color thresholds for detecting yellow blobs.
- **Blur**: Amount of blur applied to the image to reduce noise.
- **Blob Parameters**: Parameters for configuring the blob detector.
- **Detection Window**: Specifies the region of the image to search for blobs.

## Functions
- **`__init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None)`**: Initializes the detector with given parameters, sets up publishers and subscribers, and initializes the CV Bridge.
- **`blob_detect(self, image, hsv_min, hsv_max, blur=0, blob_params=None, search_window=None)`**: Detects blobs in the given image based on HSV thresholds and blob detection parameters.
- **`draw_keypoints(self, image, keypoints, line_color=(0,0,255))`**: Draws detected blobs on the image.
- **`draw_window(self, image, window_adim, color=(255,0,0), line=5)`**: Draws the search window on the image.
- **`draw_frame(self, image, dimension=0.3, line=2)`**: Draws X and Y axes on the image for reference.
- **`apply_search_window(self, image, window_adim=[0.0, 0.0, 1.0, 1.0])`**: Applies the search window mask to the image.
- **`blur_outside(self, image, blur=5, window_adim=[0.0, 0.0, 1.0, 1.0])`**: Blurs the regions outside the search window.
- **`get_blob_relative_position(self, image, keyPoint)`**: Calculates the relative position of a blob in the image frame.
- **`set_threshold(self, thr_min, thr_max)`**: Sets the HSV threshold values.
- **`set_blur(self, blur)`**: Sets the blur value.
- **`set_blob_params(self, blob_params)`**: Sets the blob detection parameters.
- **`callback(self, data)`**: Callback function for the image subscriber, processes the image to detect blobs and publishes the results.
