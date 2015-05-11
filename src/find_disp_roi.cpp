#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "opencv2/gpu/gpu.hpp"
#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stereo_msgs/DisparityImage.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <stereo_image_proc/DisparityConfig.h>
#include <stereo_msgs/DisparityImage.h>

using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher pub_disparity_roi_;

void callback(const stereo_msgs::DisparityImage& img)
{

  
  stereo_msgs::DisparityImagePtr disp_msg ;
  disp_msg->header         = img->header;
  disp_msg->image.header   = img->header;
  disp_msg->image.height   = img->height;
  disp_msg->image.width    = img->width;
  disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.step     = disp_msg->image.width * sizeof(float);
  disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
  disp_msg->min_disparity = 0;
  disp_msg->max_disparity = 64;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<stereo_msgs::DisparityImage> right_sub(nh, "/disparity", callback);
  pub_disparity_roi_ = nh.advertise<stereo_msgs::DisparityImage>("disparity_roi",1);

  ros::spin();

  return 0;
}
