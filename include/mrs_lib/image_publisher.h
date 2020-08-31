#ifndef _IMAGE_TRANSPORTER_
#define _IMAGE_TRANSPORTER_

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <mutex>
#include <memory>

namespace mrs_lib {

  struct ImagePubliserData{
    ImagePubliserData(const image_transport::Publisher& publisher, const std::string& topic_name, const ros::Time& last_hit)
      :
      publisher(publisher),
      topic_name(topic_name),
      last_hit(last_hit) {};
    image_transport::Publisher publisher;
    std::string topic_name;
    std::mutex pub_mutex;
    ros::Time last_hit;
  };

  class ImagePublisher{
    public:
      ImagePublisher(ros::NodeHandlePtr nh_);
      bool publish(std::string topic_name, double throttle_period, cv::Mat& image, bool bgr_order = false);
    
    private:
      std::string getEncoding(cv::Mat& input, bool bgr_order);
      bool throttle(int index, double throttle_period);

      ros::NodeHandlePtr nh;
      std::vector<std::unique_ptr<ImagePubliserData>> imagePublishers;
      std::unique_ptr<image_transport::ImageTransport> transport;
      cv_bridge::CvImage outputImage;
      sensor_msgs::ImagePtr msg;
      bool throttle_pass;
      std::mutex main_pub_mutex;


  };

}
#endif //_IMAGE_TRANSPORTER_
