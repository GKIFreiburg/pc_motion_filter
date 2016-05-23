#include "pc_motion_filter/pc_motion_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_bullet/tf2_bullet.h>
#include <bullet/LinearMath/btTransform.h>
#include <geometry_msgs/TransformStamped.h>

void pc_motion_filter::PcMotionFilter::update()
{
  ros::NodeHandle& nh_priv = getPrivateNodeHandle();
  nh_priv.getParamCached("linear_threshold", linear_throshold);
  nh_priv.getParamCached("degree_threshold", degree_threshold);
  nh_priv.getParamCached("stillness_duration", stillness_duration);
  nh_priv.getParamCached("frame_ids", frame_ids);
  angular_threshold = angles::from_degrees(degree_threshold);
  NODELET_DEBUG_STREAM("lin: "<<linear_throshold<<", ang: "<<angles::to_degrees(angular_threshold)<<", still: "<<stillness_duration);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pc_motion_filter::PcMotionFilter::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  tf_buffer.reset(new tf2_ros::Buffer());
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, nh));
  cloud_out_publisher = nh.advertise<sensor_msgs::PointCloud2>("output", 1, false);
  cloud_in_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, boost::bind(&PcMotionFilter::cloud_in_cb, this, _1));
}

void pc_motion_filter::PcMotionFilter::cloud_in_cb(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  update();
  ros::Time now = msg->header.stamp;
  ros::Time past = now;
  past.sec -= stillness_duration;
  geometry_msgs::TransformStamped difference_msg;
  btTransform difference;
  BOOST_FOREACH(std::string& frame_id, frame_ids){
    try{
      difference_msg = tf_buffer->lookupTransform(msg->header.frame_id, now, msg->header.frame_id, past, frame_id, ros::Duration(0.1));
      difference = tf2::transformToBullet(difference_msg);
      double linear_distance = difference.getOrigin().length();//sqrt(t.x*t.x+t.y*t.y+t.z*t.z);
      double angular_distance = difference.getRotation().getAngle();

      if (linear_distance > linear_throshold) {
        NODELET_DEBUG_STREAM("above linear threshold: "<<linear_distance<<" between "<<frame_id<<" and "<<msg->header.frame_id);
        return;
      }
      if (fabs(angular_distance) > angular_threshold) {
        NODELET_DEBUG_STREAM("above angular threshold: "<<angles::to_degrees(angular_distance)<<" between "<<frame_id<<" and "<<msg->header.frame_id);
        return;
      }
    } catch (tf2::TransformException &ex) {
      NODELET_WARN_STREAM_THROTTLE(1.0, "Could NOT transform "<<msg->header.frame_id<<" to "<<frame_id<<": "<< ex.what());
      //return;
    }
  }
  cloud_out_publisher.publish(msg);
}

PLUGINLIB_EXPORT_CLASS(pc_motion_filter::PcMotionFilter, nodelet::Nodelet);

