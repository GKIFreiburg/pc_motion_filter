#ifndef PC_MOTION_FILTER_H_
#define PC_MOTION_FILTER_H_

#include <nodelet/nodelet.h>
//#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

namespace pc_motion_filter {
/** \brief @b MotionFilter uses tracks given tf frames and tejects point clouds if these frames moved relative to the frame of the point cloud.
 * \author Andreas Hertle
 */
class PcMotionFilter : public nodelet::Nodelet
{
 public:
  void onInit();
 protected:
  void cloud_in_cb(const sensor_msgs::PointCloud2::ConstPtr msg);

  private:
  void update();
  double linear_throshold;
  double degree_threshold;
  double angular_threshold;
  double stillness_duration;
  std::vector<std::string> frame_ids;

  ros::Subscriber cloud_in_subscriber;
  ros::Publisher cloud_out_publisher;
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

}

#endif  //#ifndef PC_MOTION_FILTER_H_
