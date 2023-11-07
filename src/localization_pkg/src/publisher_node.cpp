#include <ros/ros.h> //Necessary header file for using ROS in C++.
#include <tf/transform_listener.h> // header file for the Transform Listener, which allows us to receive and use transforms.
#include <geometry_msgs/PoseStamped.h>



int main(int argc, char** argv){

//----------------------------------------------------------------------------------------------

    ros::init(argc, argv, "goal_publisher"); 
    //Initializes the ROS node with the name "goal_publisher".
    
    ros::NodeHandle nh; 
    //make node handal (nh = provides a way to interact with the ROS system.)

    tf::TransformListener istener; 
    //we will use to listen for transforms.
    
    ros::Publisher goal_publisher = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 10);
//  the publisher name = gaol_publisher
//  gaol_publisher = geometry_msgs::posestamed (to the topic)
//  topic = /gaol_pose
//  queue size = 10 (how many messisage)

    ros::Rate rate(10); // create "rate" object = 10Hz
                        // it controls loop frequency

//----------------------------------------------------------------------------------------------


    while (ros::ok()){  //this is a loop

        tf::StampedTransform transform;
        // creates "stampedTransform" object and holds transform information

        try{    // if exception of "tf::fransforException" is thrown like erreors, it catches it

            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            // look up transform between "/map" and "/base_link"
            //stores time "ros::Time(0)" in "transform" object
            
            geometry_msgs::PoseStamped goal;
            //puts position & orientation form "transform" to "goal"

            goal.header.frame_id = "map";
            // frame ID 

            goal.pose.position.x = transform.getOrigin().x();
            goal.pose.position.y = transform.getOrigin().y();
            goal.pose.position.z = transform.getOrigin().z();

            goal.pose.orientation.x = transform.getRotation().x();
            goal.pose.orientation.y = transform.getRotation().y();
            goal.pose.orientation.z = transform.getRotation().z();
            goal.pose.orientation.w = transform.getRotation().w();

            // In summary, this block of code takes information from the transform object 
            // (which represents a transformation between coordinate frames) and uses it to set the position and orientation of the goal pose in the /map coordinate frame. 
            // This goal message can then be published to the /goal_pose topic for other nodes to use.

            goal_publisher.publish(goal);
            // Publishes "goal" message to "/goal_pose" topic

        }

//-------------------------------------------------------------------------------------------------------

        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            // if there exception found, then print "error"

            ros::Duration(1.0).sleep();
            // if "error", then program sleep (1 sec)

        }

//----------------------------------------------------------------------------------------------------------

        rate.sleep();
        // Pauses for a duration for the loop to runs (10Hz)

    }
    
//-----------------------------------------------------------------------------------------

    return 0;
}

//-------------------------------------------------------------------

//             NOTES:

// ink of rod nodes and publishers: https://docs.ros.org/en/rolling/_images/Nodes-TopicandService.gif

// why initialize a rose node/create publisher ?

//     Allows your program to interact with the ROS ecosystem
//     registering your program with the ROS Master ( which is responsible for managing the communication between nodes in a ROS system.)

//             1) it give the node a name for other node to call it
//             2) alows access the ROS parameter server (holds configuration data)
//             3) prerequisite for creating publishers and subscribers.
//             4) registere in ROS network (for our program provides or consumes services or actions)
//             5) set and retrieve parameters from the parameter server
//             6) rOS logging functions to output messages to the console

// why do i need to create a transforms ?

//     understanding the spatial relationships between different components or frames of reference.
//     represent the translation and rotation between coordinate systems, allowing you to know where one object is located relative to another.

//             1) spatial awarness
//             2) where each component is in relation to a common reference frame
//             3) transforming coordinates from a sensor's frame to the robot's end-effector frame.


// why do i neede to Create a geometry_msgs/PoseStamped message ?

//       is the data structure used to represent this information. It's a standardized message type that allows you to communicate pose information.

//              1) When you set the frame ID of the PoseStamped message to /map, you're indicating that the pose information is specified in the coordinate frame of the /map frame. This is important for other nodes that receive this message to understand where the goal pose is located in the robot's global coordinate system.
//                 If the frame ID were set to something else (e.g., /base_link or /odom), it would mean that the goal pose is specified in a different coordinate frame, and nodes receiving this message would need to perform a transform to understand the pose in their own frame of reference.
//                 In summary, setting the frame ID is crucial for correctly interpreting the goal pose information, especially in the context of navigation or manipulation tasks.