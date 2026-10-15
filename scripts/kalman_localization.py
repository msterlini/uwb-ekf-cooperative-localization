import numpy as np
import rospy
from nav_msgs.msg import Odometry
from dist_project.msg import uwb_data
from geometry_msgs.msg import Pose
import tf
import time

class KalmanEstimator:

    def __init__(self):

        # Mean vector of x, y, theta initialization
        self.mu = np.array([0.0, 0.0, 0.0])

        # Covariance matrix initialization
        self.sigma = np.array([[1.0, 0.0, 0.0], 
                               [0.0, 1.0, 0.0], 
                               [0.0, 0.0, 1.0]])

        # Initialization of list that will contain the 3D position of each uwb anchor
        self.tag_pos = []
        
        # Local variable to count the number of callback to wait before starting the motion planner
        self.actual_callback = 0
        # Local variables to store the initial estimate to compare with a further one and estimate the orientation theta
        self.first_x = 0
        self.first_y = 0

        # Publisher for estimated state
        self.pub = rospy.Publisher('localization_data_topic', Pose, queue_size=10)

        # Publisher to notify the motion planner to start
        self.init_pub = rospy.Publisher('init_move', Pose, queue_size=10)

        # Subscription to odom topic directly from gazebo simulation
        rospy.Subscriber('odom', Odometry, self.subscribe_odom_data)

        # Subscription to uwb data topic directly from anchor simulation
        rospy.Subscriber('uwb_data_topic', uwb_data, self.subscribe_uwb_data)

        # Function to be called once, it scans for every uwb anchor in the area and stores their position
        self.get_anchors_pos()


    def prediction_step(self, odom_data: Odometry):
        # Local variable initialization
        x, y, theta = self.mu

        # Velocity and rotation rate extraction from Odometry data
        delta_vel = odom_data.twist.twist.linear.x
        delta_w = odom_data.twist.twist.angular.z

        # Constant noise addition
        noise = 0.1**2
        v_noise = np.random.normal(0, 0.1)*delta_vel**2
        w_noise = np.random.normal(0, 0.1)*delta_w**2

        # Covariance matrix of the process
        # I take a constant value and I add a contribute proportional to the actual value read
        Q = np.array([[noise + v_noise, 0.0], 
                      [0.0, noise + w_noise]])
        
        # Kinematic model noise redistribution
        G = np.array([[np.cos(theta), 0.0], 
                      [np.sin(theta), 0.0], 
                      [0.0, 1.0]])
        
        # Update of the state with model knowledge
        # Instead of multiplying by the delta t I divide by the odometry topic publishing frequency
        x_new = x + delta_vel * np.cos(theta) / 10
        y_new = y + delta_vel * np.sin(theta) / 10
        theta_new = (theta + delta_w / 10 + np.pi) % (2 * np.pi) - np.pi
        # Doing so I modify theta_new in order to obtain a value in [-pi, pi]

        # Jacobian of the dynamics with respect to the state
        A = np.array([[1.0, 0.0, -delta_vel * np.sin(theta) / 10],
                      [0.0, 1.0, delta_vel * np.cos(theta) / 10],
                      [0.0, 0.0, 1.0]])

        # New mean vector declaration
        self.mu = np.array([x_new, y_new, theta_new])

        # New covariance matrix
        self.sigma = (A @ self.sigma) @ A.T + (G @ Q) @ G.T

        # Now it published the estimated state
        self.publish_data(self.mu[0], self.mu[1], self.mu[2])
    
    def correction_step(self, uwb_data: uwb_data):
        # Local variable initialization
        x, y, theta = self.mu

        # UWB data unpacking
        ids = uwb_data.tag_id
        distances = uwb_data.distance

        # H and Z matrices initialization
        H = []
        Z = []
        # For expected distances I intend the result of the model of the measuring system
        expected_distances = []
        # For every lecture from every tag
        for i in range(len(ids)):
            # Actual measurement
            meas_dist = distances[i]
            lx = self.tag_pos[i][0]
            ly = self.tag_pos[i][1]
            lz = self.tag_pos[i][2]
            # I build up the H matrix that will be used later
            dist_exp = np.sqrt((lx - x)**2 + (ly - y)**2 + (lz - 0)**2)
            # I put 0 for z since the robot is bounded to the ground
            H_i = [(x - lx) / dist_exp, (y - ly) / dist_exp, 0]
            H.append(H_i)
            # I append every actual measurement to the Z matrix
            Z.append(meas_dist)
            # I build up a matrix containing all the expected measurements (H.mu) I build it line by line
            expected_distances.append(dist_exp)
        

	    # Covariance matrix of measurement process
        # As previously defined the measurements are subject to a gaussian noise whose amplitude is proportional to the acutal measurement. 
        # The matrix R will be a diagonal one with elements the parameter defined before that multiplies the actual measure squared, I add a regularization term
        R = 0.015 * np.eye(len(ids)) @ np.square(Z) + 0.5 * np.eye(len(ids))
        
        # I convert the H list I obtained before to a numpy array
        H = np.array(H)
        
        # I define the matrix S_inv as the inverse of matrix S = H.sigma.H^T + R
        S_inv = np.linalg.inv((H @ self.sigma) @ H.T + R)
        
        # I define the Kalman gain as the matrix W
        W = (self.sigma @ H.T) @ S_inv

        # Maen vector update
        self.mu = self.mu + W @ (np.array(Z) - np.array(expected_distances))

        # Sigma vector update
        self.sigma = (np.eye(len(self.sigma)) - W @ H) @ self.sigma


        # We wait for a user-designed number of callback to wait for the kalman filter to reach a good estimate starting from null mu vector
        # In this moment the robot is going in a straight line, we wait 10 more callback to get a new position and estimate theta as the result of a arctan2
        number_of_callback = 30
        if (self.actual_callback < number_of_callback + 11):
            if self.actual_callback == number_of_callback:
                # We get the first coordinates to compare
                self.first_x = self.mu[0]
                self.first_y = self.mu[1]
            elif self.actual_callback == number_of_callback + 10:
                # After 10 callbacks I compute the initial corrected theta estimate
                self.mu[2] = np.arctan2(self.mu[1] - self.first_y, self.mu[0] - self.first_x)
                # Send a message to tell the motion planner to start moving
                message = Pose()
                self.init_pub.publish(message)
            self.actual_callback += 1

        # Theta normalization to have it in [-pi, pi]
        self.mu[2] = (self.mu[2] + np.pi) % (2 * np.pi) - np.pi
    
    def subscribe_odom_data(self, odom_data: Odometry):
        # Callback function to the reception of a message from Odometry
        # With the data from the sensors on the unicyle I compute the prediction step
        self.prediction_step(odom_data)

    def subscribe_uwb_data(self, uwb_data: uwb_data):
        # Callback function to the reception of a message from the uwb anchors
        # Now that I receive the data from other sensors I perform the correction step
        self.correction_step(uwb_data)
    
    def publish_data(self, pose_x, pose_y, pose_yaw):

        # I declare the ROS msg Pose to publish
        robot_pos = Pose()
        robot_pos.position.x = float(pose_x)
        robot_pos.position.y = float(pose_y)
        robot_pos.position.z = 0

        robot_pos.orientation.z = pose_yaw
        # I use the other 3 free parameters of the orientation quaternion msg to give the uncertaintes values about x, y and theta
        # Uncertainty on x
        robot_pos.orientation.x = self.sigma[0, 0]
        # Uncertainty on y
        robot_pos.orientation.y = self.sigma[1, 1]
        # Uncertainty on theta
        robot_pos.orientation.w = self.sigma[2, 2]
        
        # Finally I publish the state
        self.pub.publish(robot_pos)

    def get_anchors_pos(self):
        # Same function can be found in uwb_dist_sim.py
        # Maximum number of anchors allowable 
        max_anchors = 20
        tag_pos = []
        # This is relative to how I declared the id of each anchor in the gazebo_init script in the publish_static_transform function
        # I called them tag_0 with increasing numbers
        uwb_id = 'uwb_anchor_'

        # I start the listener from the package tf to get the transformation between the world frame and each tag. This was a easy way to implement id dynamically without using directly the coordinate csv file. I will retrieve the informations of the anchors directly from the simulation. 
        listener = tf.TransformListener()

        try:
            for i in range(max_anchors):

                # For each possible tag name
                try:
                    time.sleep(0.5)
                    # I store the translation of each tag w.r.t. world frame
                    (trans, rot) = listener.lookupTransform('/world', uwb_id + str(i), rospy.Time(0))
                    # I store the position found
                    tag_pos.append(trans)
                
                # Error handling
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, KeyboardInterrupt):
                    break
            
                if tag_pos == []:
                    # In case no uwb anchor was detected I run the function again. Usually this won't happen since I tried to make this function run after some initialization time interval to wait for the gazebo environment to correctly set up
                    rospy.logwarn("No UWB anchor detected. Trying again")
                    self.get_anchors_pos()
                else:
                    rospy.loginfo("UWB anchor list:\n" + str(tag_pos))
                    # I update the current list of anchor positions
                    self.tag_pos = tag_pos
        
        except (KeyboardInterrupt):
            # I added this nested try-except to better catch the KeyboardInterrupt exception
            pass

if __name__ == "__main__":

    # Node initialization
    rospy.init_node('localization_kalman_filter', anonymous=True)
    
    # Parameters import
    rate_val = rospy.get_param("/localization_rate")

    # Class initialization
    estimator = KalmanEstimator()

    # Rospy rate of node impostion
    rate = rospy.Rate(rate_val)

    try:
        while not rospy.is_shutdown():
            # We don't need to cast any particular function since the estimator already works on the callbacks of the topics
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Kalman filter estimator node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("Kalman localization node interrupted")
