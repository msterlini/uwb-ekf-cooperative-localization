import numpy as np
import rospy
import tf


def get_anchors_pos(max_anchors=20):
    # This is relative to how I declared the id of each anchor in the gazebo_init script
    # I called them uwb_anchor_0 with increasing numbers
    uwb_id = 'uwb_anchor_'

    # I start the listener from the package tf to get the transformation between the world frame and each tag.
    # This was a easy way to implement id dynamically without using directly the coordinate csv file.
    # I will retrieve the informations of the anchors directly from the simulation.
    listener = tf.TransformListener()

    # I wait until the first anchor is available, the gazebo environment could still be setting up
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('world', uwb_id + '0', rospy.Time(0), rospy.Duration(1.0))
            break
        except tf.Exception:
            rospy.logwarn("No UWB anchor detected. Trying again")

    tag_pos = []
    for i in range(max_anchors):
        # For each possible tag name I store the translation w.r.t. world frame, the first missing one ends the list
        try:
            listener.waitForTransform('world', uwb_id + str(i), rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('world', uwb_id + str(i), rospy.Time(0))
        except tf.Exception:
            break
        tag_pos.append(trans)

    rospy.loginfo("UWB anchor list:\n" + str(tag_pos))
    return np.array(tag_pos)
