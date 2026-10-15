import rospy
from geometry_msgs.msg import Twist, Point, Pose

# Constants
TARGET_REACHED_Y_THRESHOLD = -0.5
PID_KP = 1.5
PID_KI = 0
PID_KD = 0.1
FORWARD_SPEED = 0.2
TURN_SPEED = 0.5

# The idea was to create a finite state machine that switch between
# INIT -> the robot goes forward in one direction for a given amount of time until the estimation of its orientation is good enough since we don't have previous information on it
# SEARCH -> In this state the robot turns on itself until it spots the target
# APPROACH -> In this state the robot approaches the target and corrects the orientation with a simple PID controller
# TARGET_REACHED -> In this state the robots stops moving and starts publishing the target height seen from its own point of view in its own camera reference frame

class MotionPlanner:

    def __init__(self):

        # Variable to hold the target height
        self.target_height = 0
        # Variable to hold current state
        self.current_state = "INIT"
        # Variables for PD control
        self.integral = 0
        self.previous_error = 0

        # Topic to publish control input
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        rospy.loginfo(f">>>> Publishing velocity to local /cmd_vel topic =====")

        # Topic to get target position of detected blob
        self.sub = rospy.Subscriber("target/point_blob", Point, self.blob_callback)
        rospy.loginfo(f"<<<< Subscribed to local target/point_blob topic =====")

        # Topic to receive switch state input from localization module to start the movement
        self.init_sub = rospy.Subscriber("init_move", Pose, self.init_callback)
        rospy.loginfo(f"<<<< Subscribed to local init_move topic =====")
    
    def init_callback(self, data):
        # In this simple callback we unregister from the init_move topic and switch the state, it will be invoked only once
        self.current_state = "SEARCH"
        self.init_sub.unregister()

    def blob_callback(self, data):
        # This callback is called everytime a target is detected from the camera
        x, y = data.x, data.y
        # Since we want to point directly at the target the x coordinate of the detected target in the camera ref. frame is the error
        error = -x

        # We act only if we are currently searching or moving towards a target
        if self.current_state == "SEARCH" or self.current_state == "APPROACH":
            # The robot stops moving towards the target once it is close enough, this is translated in the y coordinate of the target being above a certain value chosen a priori, the sign is inverted since we are referring to the camera reference frame
            if y < TARGET_REACHED_Y_THRESHOLD:
                # State switch
                self.current_state = "TARGET_REACHED"
                # Update of variable holding target_height
                self.target_height = y
                self.sub.unregister()
                # It starts publishing the height of the target that the robot sees from its point of view
                self.pub_target = rospy.Publisher("target_height", Point, queue_size=1)
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
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.pub.publish(command)
    
    # This is the finite state machine controller that will be invoked continously
    def execute_state_machine(self):
        if self.current_state == "INIT":
            # This is the command to slowly move forward until we have a good estimate of the orientation
            self.publish_velocity(0.1, 0)
        elif self.current_state == "SEARCH":
            # If the robot is still searching for a target it's only commanded to turn on the spot
            self.publish_velocity(0, TURN_SPEED)
        elif self.current_state == "TARGET_REACHED":
            # If the target is reached the robot stops and continuosly publishes the target height
            self.publish_velocity(0, 0)
            message = Point()
            message.z = self.target_height
            self.pub_target.publish(message)

if __name__ == "__main__":
    
    # Node initialization
    rospy.init_node("motion_planner", anonymous=True)

    # Parameters import
    rate_val = rospy.get_param("/motion_planner_rate")

    # Class initialilzation
    motion_planner = MotionPlanner()

    # Ropsy rate of node imposition
    rate = rospy.Rate(rate_val)

    try:
        while not rospy.is_shutdown():
            # Main loop where the finite state machine controller is invoked
            motion_planner.execute_state_machine()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Motion planner node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("Motion planner node interrupted")



        