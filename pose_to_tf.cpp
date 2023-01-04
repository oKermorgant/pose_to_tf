#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

// supported incoming poses
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using std_msgs::msg::Header;
using namespace std;

namespace pose_to_tf
{
using nav_msgs::msg::Odometry;

class Pose2TF : public rclcpp::Node
{
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::msg::TransformStamped tf;

  rclcpp::SubscriptionBase::SharedPtr pose_sub;
  rclcpp::TimerBase::SharedPtr topic_timer;

  string topic, parent_frame, child_frame;



public:
  explicit Pose2TF(rclcpp::NodeOptions options):
    Node("pose_to_tf", options),
    br(this),
    topic         {declare_parameter<string>("topic", "pose_gt")},
    parent_frame  {declare_parameter<string>("parent_frame", "world")},
    child_frame   {declare_parameter<string>("child_frame", "base_link")}
  {
    // topic to absolute in order to find it in list
    if(topic[0] != '/')
    {
      const auto ns{string(get_namespace())};
      if(ns.size() == 1)
        topic = ns + topic;
      else
        topic = ns + "/" + topic;
    }

    topic_timer = create_wall_timer(500ms, [&](){detectTopicType();});
  }

private:

  inline void republish(const string &child_frame,
                        const Transform &pose)
  {
    tf.child_frame_id = child_frame;
    tf.transform = pose;
    br.sendTransform(tf);
  }

  inline void republish(const string &child_frame, const Quaternion &orientation)
  {
    tf.child_frame_id = child_frame;
    tf.transform.rotation = orientation;
    br.sendTransform(tf);
  }

  inline void republish(const string &child_frame, const Pose &pose)
  {
    tf.child_frame_id = child_frame;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
    br.sendTransform(tf);
  }

  void detectTopicType()
  {
    const auto topics{get_topic_names_and_types()};
    const auto info{topics.find(topic)};

    if(info == topics.end())
      return;

    topic_timer.reset();

    if(info->second.size() > 1)
    {
      RCLCPP_WARN(get_logger(), topic.c_str(), "seems to have several types of message");
    }

    const auto msg{info->second[0]};

    if(msg == "geometry_msgs/msg/Pose")
    {
      pose_sub = create_subscription<Pose>(topic, 1, [&](Pose::SharedPtr msg)
      {
          tf.header.frame_id = parent_frame;
          tf.header.stamp = get_clock()->now();
          republish(child_frame, *msg);});
    }
    else if(msg == "geometry_msgs/msg/pose_stamped")
    {
      pose_sub = create_subscription<PoseStamped>(topic, 1, [&](PoseStamped::SharedPtr msg)
      {
          tf.header.frame_id = parent_frame;
          tf.header.stamp = msg->header.stamp;
          republish(msg->header.frame_id, msg->pose);});
    }
    else if(msg == "geometry_msgs/msg/Transform")
    {
      pose_sub = create_subscription<Transform>(topic, 1, [&](Transform::SharedPtr msg)
      {
          tf.header.frame_id = parent_frame;
          tf.header.stamp = get_clock()->now();
          republish(child_frame, *msg);});
    }
    else if(msg == "geometry_msgs/msg/TransformStamped")
    {
      pose_sub = create_subscription<TransformStamped>(topic, 1, [&](TransformStamped::SharedPtr msg)
      {
          tf.header = msg->header;
          republish(msg->child_frame_id, msg->transform);});
    }
    else if(msg == "nav_msgs/msg/Odometry")
    {
      pose_sub = create_subscription<Odometry>(topic, 1, [&](Odometry::SharedPtr msg)
      {
          tf.header = msg->header;
          republish(msg->child_frame_id, msg->pose.pose);});
    }
    else if(msg == "sensor_msgs/msg/Imu")
    {
      pose_sub = create_subscription<Imu>(topic, rclcpp::SensorDataQoS(), [&](Imu::SharedPtr msg)
      {
          tf.header.stamp = msg->header.stamp;
          republish(msg->header.frame_id, msg->orientation);});
    }
    else
    {
      RCLCPP_ERROR(get_logger(), topic.c_str(), " has unsupported message type ", msg.c_str());
    }
  }




};
}

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pose_to_tf::Pose2TF>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
