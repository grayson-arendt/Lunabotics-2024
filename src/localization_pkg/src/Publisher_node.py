#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def publish_goal():
    rospy.init_node('goal_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz, adjust as needed

    # Create a TF listener
    listener = tf.TransformListener()

    # Create a publisher for the goal
    goal_publisher = rospy.Publisher('/goal_pose', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        try:
            # Look up the transform from /map to /base_link
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            # Create a PoseStamped message
            goal = PoseStamped()
            goal.header.frame_id = 'map'

            goal.pose.position.x = trans[0]
            goal.pose.position.y = trans[1]
            goal.pose.position.z = trans[2]

            goal.pose.orientation.x = rot[0]
            goal.pose.orientation.y = rot[1]
            goal.pose.orientation.z = rot[2]
            
            goal.pose.orientation.w = rot[3]

            # Publish the goal
            goal_publisher.publish(goal)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass 