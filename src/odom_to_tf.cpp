#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class OdomToTF : public rclcpp::Node {
    public:
        OdomToTF() : Node("odom_to_tf") {
            std::string odom_topic;
            frame_id = this->declare_parameter("frame_id", std::string(""));
            child_frame_id = this->declare_parameter("child_frame_id", std::string(""));
            odom_topic = this->declare_parameter("odom_topic", std::string("/odom/perfect"));
            RCLCPP_INFO(this->get_logger(), "odom_topic set to %s", odom_topic.c_str());
            inverse_tf = this->declare_parameter("inverse_tf", false);

            if (frame_id != "") {
                RCLCPP_INFO(this->get_logger(), "frame_id set to %s", frame_id.c_str());
            }
            else {
                RCLCPP_INFO(this->get_logger(), "frame_id was not set. The frame_id of the odom message will be used.");
            }
            if (child_frame_id != "") {
                RCLCPP_INFO(this->get_logger(), "child_frame_id set to %s", child_frame_id.c_str());
            }
            else {
                RCLCPP_INFO(this->get_logger(), "child_frame_id was not set. The child_frame_id of the odom message will be used.");
            }
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(), std::bind(&OdomToTF::odomCallback, this, _1));
            tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
    private:
        std::string frame_id, child_frame_id;
        bool inverse_tf;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const {

            geometry_msgs::msg::TransformStamped tfs_;
            tfs_.header = msg->header;
            tfs_.header.stamp = this->now();
            if (! inverse_tf) {            
                tfs_.header.frame_id = frame_id != "" ? frame_id : tfs_.header.frame_id;
                tfs_.child_frame_id = child_frame_id != "" ? child_frame_id : msg->child_frame_id;
                tfs_.transform.translation.x = msg->pose.pose.position.x;
                tfs_.transform.translation.y = msg->pose.pose.position.y;
                tfs_.transform.translation.z = msg->pose.pose.position.z;

                tfs_.transform.rotation = msg->pose.pose.orientation;
            }
            else {
                tf2::Quaternion rot_q;
                tf2::Vector3 trans;
                tf2::fromMsg(msg->pose.pose.orientation, rot_q);
                tf2::fromMsg(msg->pose.pose.position, trans);
                tf2::Transform tf2_tf = tf2::Transform(rot_q, trans);
                tfs_.transform = tf2::toMsg(tf2_tf.inverse());
                tfs_.header.frame_id = frame_id != "" ? frame_id : msg->child_frame_id;
                tfs_.child_frame_id = child_frame_id != "" ? child_frame_id : tfs_.header.frame_id;
            }
            tfb_->sendTransform(tfs_);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTF>());
    rclcpp::shutdown();
    return 0;
}
