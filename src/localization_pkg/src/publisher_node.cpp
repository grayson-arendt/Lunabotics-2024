#include <ros/ros.h> //Necessary header file for using ROS in C++.
#include <tf/transform_listener.h> // header file for the Transform Listener, which allows us to receive and use transforms.
#include <geometry_msgs/PoseStamped.h>



int main(int argc, char** argv){
    ros::init(argc, argv, "goal_publisher"); //Initializes the ROS node with the name "goal_publisher".
    ros::NodeHandle nh; //make node handal (nh = provides a way to interact with the ROS system.)

    tf::TransformListener istener; //we will use to listen for transforms.
    ros::Publisher goal_publisher = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 10);
            // the publisher name = gaol_publisher
            // it publisher = geometry_msgs::posestamed (to the topic)
            // topic = /gaol_pose
            //queue size = 10
            

    ros::Rate rate(10); // create "rate" object = 10Hz
                        // it controls loop frequency

    while (ros::ok()){  //this is a loop
        tf::StampedTransform transform;
        // creates "stampedTransform" object and holds transform information

        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.pose.position.x = transform.getOrigin().x();
            goal.pose.position.y = transform.getOrigin().y();
            goal.pose.position.z = transform.getOrigin().z();
            goal.pose.orientation.x = transform.getRotation().x();
            goal.pose.orientation.y = transform.getRotation().y();
            goal.pose.orientation.z = transform.getRotation().z();
            goal.pose.orientation.w = transform.getRotation().w();

            goal_publisher.publish(goal);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        rate.sleep();
    }
    return 0;
}
