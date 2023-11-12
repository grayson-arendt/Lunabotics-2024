#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations
import time

#    pose = PoseStamped()
#    pose.header.frame_id = 'map'
#    pose.header.stamp = navigator.get_clock().now().to_msg()
#    pose.pose.position.x = position_x
#    pose.pose.position.y = position_y
#    pose.pose.position.z = 0.0
#    pose.pose.orientation.x = 0.0
#    pose.pose.orientation.y = 0.0
#    pose.pose.orientation.z = 0.0
#    pose.pose.orientation.w = 1.0


def main():
    # --- Init ROS2 communications and Simple Commander API ---
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose ---
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 0.0
    nav.setInitialPose(initial_pose)
    
    # --- Wait for Nav2 ---
    nav.waitUntilNav2Active(localizer="bt_navigator")
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.5
    goal_pose.pose.position.y = 2.0
    goal_pose.pose.orientation.w = 1.0
    
    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            print(feedback)
            time.sleep(2.0)
            
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')   
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()