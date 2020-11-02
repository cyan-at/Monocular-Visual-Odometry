// copyright

#ifndef POSESTAMPED_PUBLISHER_HPP_
#define POSESTAMPED_PUBLISHER_HPP_

#include <condition_variable>  // NOLINT(build/c++11)
#include "my_slam/utils/ring_buffer.hpp"

#include "my_slam/basics/opencv_funcs.h"

#include <opencv2/core/core.hpp>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class PosestampedPublisher {
 private:
  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  std::string topic_name_;
  std::string frame_id_;

  geometry_msgs::PoseStamped msg_;
  ros::Publisher publisher_;
  Quaternion quat;

 public:
  std::mutex mutex;
  std::condition_variable cv;
  utils::RingBuffer<cv::Mat> buffer;

  explicit PosestampedPublisher(
    std::shared_ptr<ros::NodeHandle> node_handle_ptr,
    std::string topic_name,
    std::string frame_id,
    int buffer_size = 1);

  virtual ~PosestampedPublisher() {}

  void run(void);
};

#endif  // POSESTAMPED_PUBLISHER_HPP_
