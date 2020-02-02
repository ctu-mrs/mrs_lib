#include <mrs_lib/image_publisher.h>

namespace mrs_lib {
  /* Constructor //{ */
  ImagePublisher::ImagePublisher(ros::NodeHandlePtr nh_){
    nh = nh_;
    transport = std::make_unique<image_transport::ImageTransport>(*nh);
  }
  //}

  /* publish //{ */
  bool ImagePublisher::publish(std::string topic_name, double throttle_period, cv::Mat& image, bool bgr_order){
    std::scoped_lock lock(main_pub_mutex);

    /* ROS_INFO_STREAM("curr. topic count: " << imagePublishers.size()); */
    int match_index = -1;
    for (int i=0; i<(int)(imagePublishers.size());i++){
      /* ROS_INFO_STREAM("curr. topic: " << imagePublishers[i].topic_name); */
      if ( (topic_name) == (imagePublishers.at(i)->topic_name) ) {
        match_index = i;
        break;
      }
    }

    if (throttle(match_index, throttle_period))
      return false;

    if (match_index == -1){
      ROS_INFO("[ImagePublisher]: creating new image publisher %s",topic_name.c_str());
      image_transport::Publisher new_publisher = transport->advertise("/debug_topics/"+nh->getNamespace()+"/"+topic_name,1);
      imagePublishers.push_back(std::make_unique<ImagePubliserData>(new_publisher, topic_name, ros::Time::now()));
      match_index = (int)(imagePublishers.size()) - 1;
      /* pub_mutex.push_back(); */
    }
    else
      imagePublishers.at(match_index)->last_hit = ros::Time::now();

    /* ROS_INFO("Here A"); */
    outputImage = cv_bridge::CvImage(std_msgs::Header(), getEncoding(image, bgr_order), image);
    /* ROS_INFO("Here B"); */
    /* ROS_INFO("Here C"); */
    msg = outputImage.toImageMsg();
    msg->header.stamp = ros::Time::now();
    /* ROS_INFO("Here D"); */


    try{
      std::scoped_lock lock(imagePublishers.at(match_index)->pub_mutex);
      imagePublishers.at(match_index)->publisher.publish(msg);
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
      case CV_16UC1:
        return sensor_msgs::image_encodings::MONO16;
      default:
        ROS_ERROR("[ImagePublisher]: Covnersion for the current matrix type is not yet implemented. Implement it yourself in mrs_lib/image_publisher. Sorry.");
        return "";
        break;
    }
  }
  //}

  /* throttle /{ */
  bool ImagePublisher::throttle(int index, double throttle_period){
    if (index < 0)
      return false;

    if ((ros::Time::now() - imagePublishers.at(index)->last_hit).toSec() < throttle_period)
      return true;
    else
      return false;
  }
  //}
}
