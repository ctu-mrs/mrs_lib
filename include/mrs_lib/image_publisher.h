#ifndef _IMAGE_TRANSPORTER_
#define _IMAGE_TRANSPORTER_

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <mutex>

namespace mrs_lib {

  class ImagePublisher{
    public:
      ImagePublisher(ros::NodeHandlePtr nh_);
      bool publish(std::string topic_name, double throttle_period, cv::Mat& image, bool bgr_order = false);
    
    private:
      std::string getEncoding(cv::Mat& input, bool bgr_order);

      ros::NodeHandlePtr nh;
      std::vector<image_transport::Publisher> imagePublishers;
      std::vector<std::mutex*> pub_mutex;
      image_transport::ImageTransport* transport;
      cv_bridge::CvImage outputImage;
      sensor_msgs::ImagePtr msg;


  };

}
#endif //_IMAGE_TRANSPORTER_
