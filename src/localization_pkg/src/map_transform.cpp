#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticMapFramePublisher : public rclcpp::Node
{
public:
    explicit StaticMapFramePublisher(char * transformation[])
    : Node("static_map_tf2_broadcaster")
    {
        tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once at startup
        this->make_map_transforms(transformation);
    }
private:
    void make_map_transforms(char * transformation[])
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = transformation[1];

        t.transform.translation.x = atof(transformation[2]);
        t.transform.translation.y = atof(transformation[3]);
        t.transform.translation.z = atof(transformation[4]);
        tf2::Quaternion q;
        q.setRPY(
        atof(transformation[5]),
        atof(transformation[6]),
        atof(transformation[7]));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
};

int main(int argc, char * argv[])
{
    // Pass parameters and initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticMapFramePublisher>(argv));
    rclcpp::shutdown();
    return 0;
}