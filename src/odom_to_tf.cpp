#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <stdexcept>

using std::placeholders::_1;

class OdomToTF : public rclcpp::Node
{
public:
  OdomToTF() : Node("odom_to_tf")
  {
    std::string odom_topic;
    frame_id_ = this->declare_parameter("frame_id", std::string(""));
    child_frame_id_ = this->declare_parameter("child_frame_id", std::string(""));
    odom_topic = this->declare_parameter("odom_topic", std::string("/odom/perfect"));
    RCLCPP_INFO(this->get_logger(), "odom_topic set to %s", odom_topic.c_str());
    inverse_tf_ = this->declare_parameter("inverse_tf", false);

    // NOTE: Deprecated parameter: declared without a default so we can detect whether it was set.
    rcl_interfaces::msg::ParameterDescriptor deprecated_desc;
    deprecated_desc.description = "DEPRECATED (inverted logic). Use use_original_odom_timestamp instead.";
    deprecated_desc.dynamic_typing = true;
    this->declare_parameter("use_original_timestamp", rclcpp::ParameterValue{}, deprecated_desc);
    if (this->get_parameter("use_original_timestamp").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "The parameter 'use_original_timestamp' is deprecated because its logic was inverted. "
                   "Use 'use_original_odom_timestamp' instead (true = use the timestamp of the odom message, "
                   "false = use the current time).");
      throw std::runtime_error("Deprecated parameter 'use_original_timestamp' is set");
    }

    use_original_odom_timestamp_ = this->declare_parameter("use_original_odom_timestamp", true);

    if (frame_id_ != "")
    {
      RCLCPP_INFO(this->get_logger(), "frame_id set to %s", frame_id_.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "frame_id was not set. The frame_id of "
                                      "the odom message will be used.");
    }
    if (child_frame_id_ != "")
    {
      RCLCPP_INFO(this->get_logger(), "child_frame_id set to %s", child_frame_id_.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "child_frame_id was not set. The child_frame_id of the odom "
                                      "message will be used.");
    }
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(),
                                                              std::bind(&OdomToTF::odomCallback, this, _1));
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  std::string frame_id_, child_frame_id_;
  bool inverse_tf_, use_original_odom_timestamp_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    geometry_msgs::msg::TransformStamped tfs_;
    if (use_original_odom_timestamp_)
    {
      tfs_.header.stamp = msg->header.stamp;
    }
    else
    {
      tfs_.header.stamp = this->now();
    }
    if (not inverse_tf_)
    {
      tfs_.header.frame_id = frame_id_ != "" ? frame_id_ : msg->header.frame_id;
      tfs_.child_frame_id = child_frame_id_ != "" ? child_frame_id_ : msg->child_frame_id;
      tfs_.transform.translation.x = msg->pose.pose.position.x;
      tfs_.transform.translation.y = msg->pose.pose.position.y;
      tfs_.transform.translation.z = msg->pose.pose.position.z;

      tfs_.transform.rotation = msg->pose.pose.orientation;
    }
    else
    {
      tfs_.header.frame_id = frame_id_ != "" ? frame_id_ : msg->child_frame_id;
      tfs_.child_frame_id = child_frame_id_ != "" ? child_frame_id_ : msg->header.frame_id;
      tf2::Vector3 trans;
      tf2::Quaternion rot_q;
      tf2::fromMsg(msg->pose.pose.position, trans);
      tf2::fromMsg(msg->pose.pose.orientation, rot_q);
      tf2::Transform tf2_tf = tf2::Transform(rot_q, trans);
      tfs_.transform = tf2::toMsg(tf2_tf.inverse());
    }
    tfb_->sendTransform(tfs_);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try
  {
    rclcpp::spin(std::make_shared<OdomToTF>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("odom_to_tf"), "Exiting: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
