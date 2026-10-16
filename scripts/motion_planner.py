#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from dist_project.transforms import yaw_from_quaternion

# Constants
TARGET_REACHED_Y_THRESHOLD = -0.5
PID_KP = 1.5
PID_KI = 0
PID_KD = 0.1
FORWARD_SPEED = 0.2
TURN_SPEED = 0.5
# Time after which a target that is not seen anymore is considered lost
TARGET_LOST_TIME = 1.0
# Distance travelled in the EXPLORE state before searching again
EXPLORE_DISTANCE = 1.5

# The idea was to create a finite state machine that switch between
# INIT -> the robot goes forward in one direction for a given amount of time until the estimation of its orientation is good enough since we don't have previous information on it
# SEARCH -> In this state the robot turns on itself until it spots the target
# APPROACH -> In this state the robot approaches the target and corrects the orientation with a simple PID controller
# TARGET_REACHED -> In this state the robots stops moving and starts publishing the target height seen from its own point of view in its own camera reference frame
# EXPLORE -> If after a full turn the target was not found the robot moves towards the center of the room and then searches again

class MotionPlanner(Node):

    def __init__(self):
        super().__init__("motion_planner", automatically_declare_parameters_from_overrides=True)

        # Variable to hold the target height
        self.target_height = 0
        # Variable to hold current state
        self.current_state = "INIT"
        # Variables for PD control
        self.integral = 0
        self.previous_error = 0
        # Time of the last detection of the target and of the start of the search
        self.last_blob_time = 0
        self.search_start_time = 0
        # Estimated pose of the robot and position where the exploration started
        self.pose = None
        self.explore_start = None
        # Point towards which the robot explores, the center of the room
        self.explore_point = np.array([self.get_parameter("explore_point_x").value,
                                       self.get_parameter("explore_point_y").value])

        # Topic to publish control input
        self.pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.get_logger().info(">>>> Publishing velocity to local /cmd_vel topic =====")

        # Topic to get target position of detected blob
        self.sub = self.create_subscription(Point, "target/point_blob", self.blob_callback, 10)
        self.get_logger().info("<<<< Subscribed to local target/point_blob topic =====")

        # Topic to receive switch state input from localization module to start the movement
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.init_sub = self.create_subscription(Pose, "init_move", self.init_callback, latched)
        self.get_logger().info("<<<< Subscribed to local init_move topic =====")

        # Topic of the estimated pose, used to move towards the center of the room while exploring
        self.create_subscription(PoseWithCovarianceStamped, "localization_data_topic", self.localization_callback, 1)

    def init_callback(self, data):
        # In this simple callback we unregister from the init_move topic and switch the state, it will be invoked only once
        self.start_search()
        self.destroy_subscription(self.init_sub)

    def start_search(self):
        self.current_state = "SEARCH"
        self.search_start_time = self.time()

    def time(self):
        # Simulation time in seconds
        return self.get_clock().now().nanoseconds * 1e-9

    def localization_callback(self, data):
        pose = data.pose.pose
        q = pose.orientation
        theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.pose = np.array([pose.position.x, pose.position.y, theta])

        if self.current_state == "EXPLORE":
            self.explore()

    def explore(self):
        # Once the robot travelled enough or it is close to the center of the room it searches again
        travelled = np.linalg.norm(self.pose[:2] - self.explore_start)
        to_point = self.explore_point - self.pose[:2]
        if travelled > EXPLORE_DISTANCE or np.linalg.norm(to_point) < 0.5:
            self.publish_velocity(0, 0)
            self.start_search()
            return

        # Heading error towards the center of the room, in [-pi, pi]
        error = (np.arctan2(to_point[1], to_point[0]) - self.pose[2] + np.pi) % (2 * np.pi) - np.pi

        # As for the approach the robot moves forward only if it is already pointing in the right direction
        if abs(error) > 0.2:
            self.publish_velocity(0, np.clip(PID_KP * error, -TURN_SPEED, TURN_SPEED))
        else:
            self.publish_velocity(FORWARD_SPEED, PID_KP * error)

    def blob_callback(self, data):
        # This callback is called everytime a target is detected from the camera
        x, y = data.x, data.y
        # Since we want to point directly at the target the x coordinate of the detected target in the camera ref. frame is the error
        error = -x

        # We act only if we are currently searching, exploring or moving towards a target
        if self.current_state in ("SEARCH", "EXPLORE", "APPROACH"):
            self.last_blob_time = self.time()
            # The robot stops moving towards the target once it is close enough, this is translated in the y coordinate of the target being above a certain value chosen a priori, the sign is inverted since we are referring to the camera reference frame
            if y < TARGET_REACHED_Y_THRESHOLD:
                # State switch
                self.current_state = "TARGET_REACHED"
                # Update of variable holding target_height
                self.target_height = y
                self.destroy_subscription(self.sub)
                # It starts publishing the height of the target that the robot sees from its point of view
                self.pub_target = self.create_publisher(Point, "target_height", 1)
            else:
                # If the robot is not close enough we are still approaching the target
                self.current_state = "APPROACH"
                # I call a function that holds the control logic of a simple PID controller
                self.move_towards_target(error)

    def move_towards_target(self, error):
        # This functions manages the control input of the robot, it just takes as input the error

        # Update integral and derivative values
        self.integral += error
        derivative = error - self.previous_error

        # Simple logic
        control_input = PID_KP * error + PID_KI * self.integral + PID_KD * derivative
        # Update of previous error with the current value
        self.previous_error = error

        # I decided to move forward only if we are pointing already the target, otherwise the robot only turns
        linear_velocity = FORWARD_SPEED if abs(error) < 0.1 else 0
        # I publish to the cmd_vel topic the linear velocity and rotational input that the gazebo plugin will manage in order to command the robot to move in the simulation
        self.publish_velocity(linear_velocity, control_input)

    def publish_velocity(self, linear_x, angular_z):
        # The cmd_vel topic accepts messages of type Twist
        command = Twist()
        command.linear.x = float(linear_x)
        command.angular.z = float(angular_z)
        self.pub.publish(command)

    # This is the finite state machine controller that will be invoked continously
    def execute_state_machine(self):
        if self.current_state == "INIT":
            # This is the command to slowly move forward until we have a good estimate of the orientation
            self.publish_velocity(0.1, 0)
        elif self.current_state == "SEARCH":
            if self.time() - self.search_start_time > 1.2 * 2 * np.pi / TURN_SPEED and self.pose is not None:
                # After a full turn without seeing the target the robot moves towards the center of the room
                self.current_state = "EXPLORE"
                self.explore_start = self.pose[:2].copy()
            else:
                # If the robot is still searching for a target it's only commanded to turn on the spot
                self.publish_velocity(0, TURN_SPEED)
        elif self.current_state == "APPROACH":
            # The gazebo plugin keeps applying the last command, if the target is lost the robot must stop and search
            # again instead of going straight until it hits a wall
            if self.time() - self.last_blob_time > TARGET_LOST_TIME:
                self.publish_velocity(0, 0)
                self.start_search()
        elif self.current_state == "TARGET_REACHED":
            # If the target is reached the robot stops and continuosly publishes the target height
            self.publish_velocity(0, 0)
            message = Point()
            message.z = float(self.target_height)
            self.pub_target.publish(message)

if __name__ == "__main__":

    # Node initialization
    rclpy.init()

    # Class initialilzation
    motion_planner = MotionPlanner()

    # The finite state machine controller is invoked at a fixed rate
    motion_planner.create_timer(1.0 / motion_planner.get_parameter("motion_planner_rate").value,
                                motion_planner.execute_state_machine)

    try:
        rclpy.spin(motion_planner)
    except KeyboardInterrupt:
        motion_planner.get_logger().info("Program shutdown: Motion planner node interrupted")
