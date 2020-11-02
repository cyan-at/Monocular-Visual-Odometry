// copyright

#include "my_slam/ros1/posestamped_publisher.hpp"

PosestampedPublisher::PosestampedPublisher(
  std::shared_ptr<ros::NodeHandle> node_handle_ptr,
  std::string topic_name,
  std::string frame_id,
  int buffer_size)
  : buffer(buffer_size),
  node_handle_ptr_(node_handle_ptr),
  topic_name_(topic_name),
  frame_id_(frame_id) {

  publisher_ = node_handle_ptr_->advertise<
    geometry_msgs::PoseStamped>(
    topic_name_, 1);

  msg_.header.frame_id = frame_id_;

  // initialize at origin / no rotation
  msg_.pose.position.x = 0.0;
  msg_.pose.position.y = 0.0;
  msg_.pose.position.z = 0.0;
  msg_.pose.orientation.w = 1.0;
  msg_.pose.orientation.x = 0.0;
  msg_.pose.orientation.y = 0.0;
  msg_.pose.orientation.z = 0.0;
}

void PosestampedPublisher::run(void) {
  while (ros::ok()
    && ros::master::check()
    && !ros::isShuttingDown()) {
    std::unique_lock<std::mutex> lock(mutex);
    // unique_lock can be unlocked explicitly
    // unlike lock_guard which only unlocks on dtor

    // predicate: wait for ring buffer update
    while (buffer.empty()) {
      cv.wait(lock);
      if (!ros::ok() || !ros::master::check() || ros::isShuttingDown()) { return; }
    }

    cv::Mat consumable = buffer.back();

    try {
      buffer.pop_back();
    } catch (utils::RingBufferException e) {
      ROS_WARN("invalid awakening!\n");
      lock.unlock();
    continue;
    }

    lock.unlock();

    // type conversion here
    // TODO(5207) define type-conversion in class?
    // which is analogous to event::deserialize

    // internal semantics:
    // produce update to publisher
    msg_.pose.position.x =
      consumable.at<double>(0, 3);
    msg_.pose.position.y =
      consumable.at<double>(1, 3);
    msg_.pose.position.z =
      consumable.at<double>(2, 3);

    my_slam::basics::getQuaternion(consumable, &quat);
    msg_.pose.orientation.x =
        quat.X;
    msg_.pose.orientation.y =
        quat.Y;
    msg_.pose.orientation.z =
        quat.Z;
    msg_.pose.orientation.w =
        quat.W;
    publisher_.publish(msg_);
  }
}