#include <functional>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class TfToOdometry : public rclcpp::Node
{
public:
  TfToOdometry() : Node("voxel_tf_to_odometry")
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/voxel_odom", 10);
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
      std::bind(&TfToOdometry::on_tf, this, std::placeholders::_1));
  }

private:
  void on_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & transform : msg->transforms) {
      if (transform.header.frame_id != "camera_init" ||
          transform.child_frame_id != "aft_mapped") {
        continue;
      }

      nav_msgs::msg::Odometry odom;
      odom.header = transform.header;
      odom.child_frame_id = transform.child_frame_id;
      odom.pose.pose.position.x = transform.transform.translation.x;
      odom.pose.pose.position.y = transform.transform.translation.y;
      odom.pose.pose.position.z = transform.transform.translation.z;
      odom.pose.pose.orientation = transform.transform.rotation;

      // TF contains no velocity or covariance. Use large variances so consumers
      // do not treat the placeholder zero twist as a precise measurement.
      for (int i = 0; i < 6; ++i) {
        odom.pose.covariance[i * 6 + i] = 1e6;
        odom.twist.covariance[i * 6 + i] = 1e6;
      }

      odom_pub_->publish(odom);
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfToOdometry>());
  rclcpp::shutdown();
  return 0;
}
