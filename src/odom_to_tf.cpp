#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class OdomToTF : public rclcpp::Node {
    public:
        OdomToTF() : Node("odom_to_tf") {
            std::string odom_topic;
            this->get_parameter_or("odom_topic", odom_topic, std::string("/odom/perfect"));
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(), std::bind(&OdomToTF::odomCallback, this, _1));
            tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const {

            geometry_msgs::msg::TransformStamped tfs_;
            tfs_.header = msg->header;
            tfs_.child_frame_id = msg->child_frame_id;
            tfs_.transform.translation.x = msg->pose.pose.position.x;
            tfs_.transform.translation.y = msg->pose.pose.position.y;
            tfs_.transform.translation.z = msg->pose.pose.position.z;

            tfs_.transform.rotation = msg->pose.pose.orientation;

            tfb_->sendTransform(tfs_);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTF>());
    rclcpp::shutdown();
    return 0;
}
