#include <mrs_lib/image_publisher.h>

/* MACROS //{ */
#define IMPUB_DEFINE_LOCATION(cond ) \
  static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, NULL}; /* Initialized at compile-time */ \
  if (ROS_UNLIKELY(!__rosconsole_define_location__loc.initialized_)) \
  { \
    initializeLogLocation(&__rosconsole_define_location__loc, "IMPUB", ::ros::console::levels::Info); \
  } \
  if (ROS_UNLIKELY(__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info)) \
  { \
    setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); \
    checkLogLocationEnabled(&__rosconsole_define_location__loc); \
  } \
  bool __impub_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (cond);

#define IMPUB_THROTTLE(period, ...) \
  do \
  { \
    IMPUB_DEFINE_LOCATION(true); \
    static double __impub_throttle_last_hit__ = 0.0; \
    double __impub_throttle_now__ = ::ros::Time::now().toSec(); \
    if (ROS_UNLIKELY(__impub_define_location__enabled) && ROSCONSOLE_THROTTLE_CHECK(__impub_throttle_now__, __impub_throttle_last_hit__, period))\
    { \
      __impub_throttle_last_hit__ = __impub_throttle_now__; \
    } \
  } while(false)

//}

namespace mrs_lib {
  /* Constructor //{ */
  ImagePublisher::ImagePublisher(ros::NodeHandlePtr nh_){
    nh = nh_;
    transport = new image_transport::ImageTransport(*nh);
  }
  //}

  /* publish //{ */
  bool ImagePublisher::publish(std::string topic_name, double throttle_period, cv::Mat& image, bool bgr_order){
    int match_index = -1;
    for (int i=0; i<(int)(imagePublishers.size());i++){
      /* ROS_INFO_STREAM("curr. topic: " << imagePublishers[i].getTopic()); */
      if ( ("/"+topic_name) == imagePublishers[i].getTopic() ) {
        match_index = i;
        break;
      }
    }

    if (match_index == -1){
      ROS_INFO("[ImagePublisher]: creating new image publisher %s",topic_name.c_str());
      image_transport::Publisher new_publisher = transport->advertise(topic_name,1);
      imagePublishers.push_back(new_publisher);
      match_index = (int)(imagePublishers.size()) - 1;
      pub_mutex.push_back(new std::mutex);
    }

    /* ROS_INFO("Here A"); */
    outputImage = cv_bridge::CvImage(std_msgs::Header(), getEncoding(image, bgr_order), image);
    /* ROS_INFO("Here B"); */
    /* ROS_INFO("Here C"); */
    msg = outputImage.toImageMsg();
    msg->header.stamp = ros::Time::now();
    /* ROS_INFO("Here D"); */

    IMPUB_THROTTLE(throttle_period);
    try{
      std::scoped_lock lock(*pub_mutex[match_index]);
      imagePublishers[match_index].publish(msg);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("[ImagePublisher]: error msg " << e.what());
      return false;
    }
    return true;
  }
  //}

  /* getEncoding //{ */
  std::string ImagePublisher::getEncoding(cv::Mat& input, bool bgr_order){
    switch (input.type()){
      case CV_8UC1:
        return sensor_msgs::image_encodings::MONO8;
        break;
      case CV_8UC3:
        if (bgr_order)
          return sensor_msgs::image_encodings::BGR8;
        else
          return sensor_msgs::image_encodings::RGB8;
      default:
        ROS_ERROR("[ImagePublisher]: Covnersion for the current matrix type is not yet implemented. Implement it yourself in mrs_lib/image_publisher. Sorry.");
        return "";
        break;
    }
  }
  //}
}
