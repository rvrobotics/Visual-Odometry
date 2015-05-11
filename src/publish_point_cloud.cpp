
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
ros::Publisher pub_disparity_;
ros::Publisher pub_disparity_roi_;

int preFilterSize = 5;
int preFilterCap = 15;
int SADWindowSize = 5;
int minDisparity = 0;
int numberOfDisparities = 48;//256
int uniquenessRatio = 0;
int speckleWindowSize = 100;
int speckleRange = 32;
int disp12MaxDiff = 0;//1
int fullDP = 0;//1
int P1 = 50;
int P2 = 800;

void callback(const ImageConstPtr& left,const ImageConstPtr& right)
{
  

  stereo_msgs::DisparityImagePtr disp_msg= boost::make_shared<stereo_msgs::DisparityImage>();
  stereo_msgs::DisparityImagePtr disp_msg1 = boost::make_shared<stereo_msgs::DisparityImage>();
  disp_msg->header         = left->header;
  disp_msg->image.header   = left->header;
  disp_msg->image.height   = left->height;
  disp_msg->image.width    = left->width;
  disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.step     = disp_msg->image.width * sizeof(float);
  disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
  disp_msg->min_disparity = 0;
    disp_msg->max_disparity = 64;
 cv::StereoSGBM block_matcher_;
  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(left, sensor_msgs::image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(right, sensor_msgs::image_encodings::MONO8)->image;
  cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                             reinterpret_cast<float*>(&disp_msg->image.data[0]),
                             disp_msg->image.step);

  disp_msg1->header         = left->header;
  disp_msg1->image.header   = left->header;
  disp_msg1->image.height   = left->height;
  disp_msg1->image.width    = left->width;
  disp_msg1->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg1->image.step     = disp_msg1->image.width * sizeof(float);
  disp_msg1->image.data.resize(disp_msg1->image.height * disp_msg1->image.step);
  disp_msg1->min_disparity = 0;
  disp_msg1->max_disparity = 64;


  cv::Mat_<float> disp_image_roi(disp_msg1->image.height, disp_msg1->image.width, reinterpret_cast<float*>(&disp_msg1->image.data[0]), disp_msg1->image.step);
  //cv::Rect_<float> myROI(250,100,400,200);
  if (1) {
    //block_matcher_.preFilterSize       = preFilterSize;
    block_matcher_.preFilterCap         = preFilterCap;
    block_matcher_.SADWindowSize       = SADWindowSize;
    block_matcher_.minDisparity        = minDisparity;
    block_matcher_.numberOfDisparities= numberOfDisparities;
    block_matcher_.uniquenessRatio     = uniquenessRatio;
    block_matcher_.speckleWindowSize   = speckleWindowSize;
    block_matcher_.speckleRange        = speckleRange;
    block_matcher_.disp12MaxDiff= disp12MaxDiff;
    block_matcher_.fullDP = fullDP;
    block_matcher_.P1 = P1;
    block_matcher_.P2 = P2;
    cv::Mat l1,r1;
    l_image.convertTo(l1,CV_8UC1);
    r_image.convertTo(r1,CV_8UC1);
    cv::Mat disp_image1;
    //cv::imshow("l1",l1);
    block_matcher_(l_image, r_image, disp_image1);
    //while(1){;}
    disp_image1.convertTo(disp_image,CV_32F);
    //disp_image.convertTo(disp_image,CV_32F);
    disp_image /= 16;
     
  }
  //disp_image_roi = disp_image(myROI);
  pub_disparity_.publish(disp_msg);
  int x1=500,y1=250,x2=700,y2=350;
  for(int i=0;i<disp_image.cols;i++){
  	for(int j=0;j<disp_image.rows;j++){
		if((x1<=i && i<=x2) && (y1<=j && j<=y2)){
			disp_image_roi.at<float>(j,i)=disp_image.at<float>(j,i);
		}
		else{
		 	disp_image_roi.at<float>(j,i)=0;
			//disp_image.at<float>(j,i)=0;
		}
	}
  }
  //cv::imshow("disparity",disp_image_roi);
  //disp_image_roi.at<float>(100,100)=0;
  //cv::copyMakeBorder(disp_image,disp_image,150,150,450,450,cv::BORDER_CONSTANT,cv::Scalar(0));
  //disp_image_roi.convertTo(disp_image_roi,CV_32F);
  pub_disparity_roi_.publish(disp_msg1);
  cv::waitKey(100);  
}

void on_trackbar( int, void* )
{
  if(SADWindowSize%2==0)
    SADWindowSize--;
  if(preFilterSize%2==0)
    preFilterSize--;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image_sub(nh, "/kitti_stereo/left/image_rect", 10);
  message_filters::Subscriber<Image> right_sub(nh, "/kitti_stereo/right/image_rect", 10);
  pub_disparity_ = nh.advertise<stereo_msgs::DisparityImage>("/kitti_stereo/disparity_mine", 1);
  pub_disparity_roi_ = nh.advertise<stereo_msgs::DisparityImage>("/kitti_stereo/disparity_roi",1);
  //typedef sync_policies::ExactTime<Image, Image> MySyncPolicy;
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  cv::namedWindow("Linear Blend", 1);
  cv::createTrackbar( "preFilterSize", "Linear Blend", &preFilterSize, 20, on_trackbar );
  cv::createTrackbar( "preFilterCap", "Linear Blend", &preFilterCap,  50, on_trackbar );
  cv::createTrackbar( "SADWindowSize", "Linear Blend", &SADWindowSize, 50, on_trackbar );
  cv::createTrackbar( "minDisparity", "Linear Blend", &minDisparity, 10, on_trackbar );
  cv::createTrackbar( "uniquenessRatio", "Linear Blend", &uniquenessRatio, 15, on_trackbar );
  cv::createTrackbar( "speckleWindowSize", "Linear Blend", &speckleWindowSize, 1000, on_trackbar );
  cv::createTrackbar( "speckleRange", "Linear Blend", &speckleRange, 35, on_trackbar );
  cv::createTrackbar( "disp12MaxDiff", "disp12MaxDiff", &disp12MaxDiff, 10, on_trackbar );
  cv::createTrackbar( "fullDP", "fullDP", &fullDP, 10, on_trackbar );
  cv::createTrackbar( "P1", "P1", &P1, 50, on_trackbar );
  cv::createTrackbar( "P2", "P2", &P2, 800, on_trackbar );
  //cv::createTrackbar( "numberOfDisparities", "Linear Blend", &numberOfDisparities, 200, on_trackbar );
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, right_sub);
  

  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
