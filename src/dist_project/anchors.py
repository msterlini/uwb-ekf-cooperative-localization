import time

import numpy as np
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def get_anchors_pos(node, max_anchors=20):
    # This is relative to how I declared the id of each anchor in the launch file
    # I called them uwb_anchor_0 with increasing numbers
    uwb_id = 'uwb_anchor_'

    # I start the listener from the package tf2 to get the transformation between the world frame and each tag.
    # This was a easy way to implement id dynamically without using directly the coordinate csv file.
    # I will retrieve the informations of the anchors directly from the simulation.
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    # I wait until the first anchor is available, the simulation could still be setting up
    while rclpy.ok() and not buffer.can_transform('world', uwb_id + '0', Time()):
        rclpy.spin_once(node, timeout_sec=0.5)
        node.get_logger().info("Waiting for the UWB anchors...", throttle_duration_sec=5.0)

    # Each anchor is published by its own static publisher, I give them some time to arrive
    deadline = time.monotonic() + 2.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    tag_pos = []
    for i in range(max_anchors):
        # For each possible tag name I store the translation w.r.t. world frame, the first missing one ends the list
        try:
            t = buffer.lookup_transform('world', uwb_id + str(i), Time()).transform.translation
        except TransformException:
            break
        tag_pos.append([t.x, t.y, t.z])

    node.get_logger().info("UWB anchor list:\n" + str(tag_pos))
    # The anchors are static, the listener is not needed anymore
    listener.unregister()
    return np.array(tag_pos)
