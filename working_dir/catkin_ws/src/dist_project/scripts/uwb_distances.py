import rospy
from std_msgs.msg import String

# Parameters import
init_time = 2

# Node initialization
rospy.init_node('uwb_simulation', anonymous=True)

pub = rospy.Publisher('test_topic', String, queue_size=10)

string = "Test\n"

rate = rospy.Rate(10)

try:
    while (not rospy.is_shutdown()):
        pub.publish(string)
except KeyboardInterrupt:
    print("Shutting down")