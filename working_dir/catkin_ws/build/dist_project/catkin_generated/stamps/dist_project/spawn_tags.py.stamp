import rospy
from gazebo_msgs.srv import SpawnModel
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import csv
import tf2_ros

# Parameters import
z_height = rospy.get_param("tag_height")
freq = rospy.get_param("tag_distance_rate")
file_dir = rospy.get_param("coordinate_file")
tag_urdf_file = rospy.get_param("tag_urdf_file")

def spawn_tag(x, y, id):
    # I use the service given by gazebo package spawn_urdf_model to dynamically spawn the tags at startup
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z_height
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        # Actual call to the service
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        # Model name to be visualized in gazebo
        spawn_model_client(
        model_name="tag_" + id,
            model_xml=open(tag_urdf_file, 'r').read(),
            initial_pose = pose,
            reference_frame = 'world'
        )
    except rospy.ServiceException as e:
        rospy.logerr("Tag initializer failed: %s", e)

def publish_static_transform(x, y, id):
    # Creation of message to be published from each tag, it will be used by the localization script to get the static positions of each anchor
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    # Frame to measure the distance of each anchor with
    static_transformStamped.header.frame_id = "world"
    # Id used to identify each anchor
    static_transformStamped.child_frame_id = "uwb_anchor_" + str(i)
    static_transformStamped.transform.translation.x = pos[0]
    static_transformStamped.transform.translation.y = pos[1]
    static_transformStamped.transform.translation.z = z_height
    static_transformStamped.transform.rotation.x = 0
    static_transformStamped.transform.rotation.y = 0
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = 1

    # Pubblication of the transformation
    broadcaster.sendTransform(static_transformStamped)

if __name__ == "__main__":
    
    # List to add the coordinates read from csv file
    positions = []
    with open(file_dir, 'r') as f:
        csvreader = csv.reader(f) 
        for row in csvreader:
            # I cast to float each coordinate value
            row = [float(x) for x in row]
            positions.append(row)

    # Initialization of the node
    rospy.init_node("tag_spawn_node")

    # Gazebo spawn
    for i, pos in enumerate(positions):
        # I call the function for each line (tag)
        spawn_tag(pos[0], pos[1], str(i))
    

    # Tf publishing 
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        for i, pos in enumerate(positions):
            publish_static_transform(pos[0], pos[1], str(i))
        rate.sleep()

