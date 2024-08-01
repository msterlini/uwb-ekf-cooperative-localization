# Scripts

These python scripts hold all the nodes and logic of the project

# `gazebo_init.py`
## Key Features
This node dynamically spawns tags, targets, robots and the room model in the Gazebo simulation environment. It also recursively initialize all the other ROS nodes for each robot based on the specified number of lines in the csv file.
It also continuosly publishes static transforms for the tags, aiding in localization tasks.

## Topics
The script interacts with the following topics:
- **Publication**: 
  - Static transforms are published for tags to the `/tf_static` topic using `tf2_ros.StaticTransformBroadcaster`.

## Functions
- **`spawn_tag(x, y, id)`**: Spawns a tag at the given coordinates using a URDF model.
- **`spawn_target(x, y, id)`**: Spawns a target at the given coordinates using an SDF model.
- **`spawn_robot(x, y, id)`**: Spawns a robot at the given coordinates using a URDF model.
- **`spawn_room()`**: Spawns the room model in the simulation using an SDF file.
- **`publish_static_transform(x, y, id)`**: Publishes the static transform for a tag.

# `uwb_dist_sim.py`

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


# `blob_detector.py`

## Key Features
It identifies yellow spheres in the image stream using color thresholding and blob detection, then it publishes the coordinates of detected blobs to a ROS topic and finally it visualizes detected blobs and the search window on the processed image.

## Topics
The script interacts with the following topics:
- **Subscription**:
  - `camera/image_raw`: Subscribes to the raw image stream from the robot's camera.
- **Publication**:
  - `target/image_blob`: Publishes the processed image with detected blobs.
  - `target/point_blob`: Publishes the coordinates of the detected blob.

## Functions
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

# `kalman_localization.py`

## Key Features

The script dynamically retrieves the positions of UWB anchors from the simulation, then it utilizes a Kalman Filter to predict and correct the robot's state, integrating odometry and UWB sensor data, it processes Odometry messages to predict the robot's movement based on velocity and angular data, it incorporates UWB sensor measurements to correct the robot's state by comparing expected distances with actual measurements, the estimated position and orientation, along with the associated uncertainties, are published to the ROS topic `localization_data_topic` and finally it notifies the motion planner to start once the Kalman Filter has stabilized its estimates.

## Topics
The script interacts with the following topics:
- **Subscription:**
  - `odom` (Odometry): Receives robot's movement data for prediction.
  - `uwb_data_topic` (uwb_data): Receives UWB sensor measurements for correction.

- **Publication:**
  - `localization_data_topic` (Pose): Publishes the estimated robot state including position, orientation, and uncertainties.
  - `init_move` (Pose): Signals the motion planner to start moving once the filter's estimates are reliable.

## Functions
- **`prediction_step(self, odom_data: Odometry)`**: Updates and predicts the robot’s state based on Odometry data and publishes the estimated state.
- **`correction_step(self, uwb_data: uwb_data)`**: Refines the robot’s state estimate using UWB data and notifies the motion planner when ready.
- **`subscribe_odom_data(self, odom_data: Odometry)`**: Callback for Odometry data, calls `prediction_step` to update the robot’s state.
- **`subscribe_uwb_data(self, uwb_data: uwb_data)`**: Callback for UWB data, calls `correction_step` to adjust the robot’s state estimate.
- **`publish_data(self, pose_x, pose_y, pose_yaw)`**: Publishes the estimated robot state and uncertainties to the ROS topic.
- **`get_anchors_pos(self)`**: Retrieves and updates UWB anchor positions from the simulation environment.

# `motion_planner.py`

## Key Features
This script manages robot behavior using a finite state machine (FSM), it uses PID control for target orientation and movement and it publishes velocity commands and target height data.

## Topics
The script interacts with the following topics:
- **Subscription:**
  - `target/point_blob` (Point): Receives target position data to adjust movement.
  - `init_move` (Pose): Receives a signal to start the FSM process.

- **Publication:**
  - `cmd_vel` (Twist): Publishes velocity commands to control the robot’s movement.
  - `target_height` (Point): Publishes the target height once reached.

## Functions
- **`init_callback(self, data)`**: Changes FSM state to "SEARCH" upon receiving initialization signal.
- **`blob_callback(self, data)`**: Updates FSM state based on detected target position and adjusts movement.
- **`move_towards_target(self, error)`**: Computes PID control and updates robot movement towards the target.
- **`publish_velocity(self, linear_x, angular_z)`**: Publishes movement commands to `cmd_vel`.
- **`execute_state_machine(self)`**: Executes FSM logic to handle state transitions and actions.

# `target_estimator.py`

## Key Features
This script estimates the target's position and height using data from multiple robots. It publishes target estimates and robot tags to facilitate collaborative localization.

## Topics
The script interacts with the following topics:
- **Subscription:**
  - `target_height` (Point): Receives target height data from local sensors.
  - `localization_data_topic` (Pose): Receives robot position and orientation data.
  - `/processing_data` (robot_data): Receives tags from other robots for target estimation.

- **Publication:**
  - `/processing_data` (robot_data): Publishes robot data to share with other robots.
  - `target_estimate` (Pose): Publishes the estimated target position and height.

## Functions
- **`target_callback(self, data)`**: Updates the target height from local sensor data.
- **`localization_callback(self, actual_pos)`**: Updates robot position and orientation; starts publishing if target height is available.
- **`least_square_estimation(self, slopes, sigma_slopes, y_intercepts, sigma_y_intercepts)`**: Estimates target position using least squares method.
- **`gather_callback(self, data)`**: Processes tags from other robots and computes target position based on collaborative data.

