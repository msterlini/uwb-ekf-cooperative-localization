import rospy
from geometry_msgs.msg import Twist, Point, Pose

# Constants
TARGET_REACHED_Y_THRESHOLD = -0.5
PID_KP = 1.5
PID_KI = 0
PID_KD = 0.1
FORWARD_SPEED = 0.2
TURN_SPEED = 0.5

class MotionPlanner:

    def __init__(self):

        # Target height variable
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
        self.current_state = "SEARCH"
        self.init_sub.unregister()

    def blob_callback(self, data):
        x, y = data.x, data.y
        error = -x

        if self.current_state == "SEARCH" or self.current_state == "APPROACH":
            if y < TARGET_REACHED_Y_THRESHOLD:
                self.current_state = "TARGET_REACHED"
                self.target_height = y
                self.sub.unregister()
                self.pub_target = rospy.Publisher("target_height", Point, queue_size=1)
            else:
                self.current_state = "APPROACH"
                self.move_towards_target(error)
    
    def move_towards_target(self, error):
        
        # Update integral and derivative values
        self.integral += error
        derivative = error - self.previous_error

        control_input = PID_KP * error + PID_KI * self.integral + PID_KD * derivative
        self.previous_error = error

        linear_velocity = FORWARD_SPEED if abs(error) < 0.1 else 0
        self.publish_velocity(linear_velocity, control_input)
    
    def publish_velocity(self, linear_x, angular_z):
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.pub.publish(command)
    
    def execute_state_machine(self):
        if self.current_state == "INIT":
            self.publish_velocity(0.1, 0)
        elif self.current_state == "SEARCH":
            self.publish_velocity(0, TURN_SPEED)
        elif self.current_state == "TARGET_REACHED":
            message = Point()
            message.z = self.target_height
            self.pub_target.publish(message)
            self.current_state = "DONE"

if __name__ == "__main__":
    
    rospy.init_node("motion_planner", anonymous=True)

    # Parameters import
    rate_val = rospy.get_param("/motion_planner_rate")

    motion_planner = MotionPlanner()

    rate = rospy.Rate(rate_val)

    try:
        while not rospy.is_shutdown():
            motion_planner.execute_state_machine()
            rate.sleep()
    except rospy.ROSInternalException:
        rospy.loginfo("Motion planner node interrupted")



        