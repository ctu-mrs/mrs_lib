#include <mrs_lib/transformer.h>

// very rudimentary usage example, more of a test
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "transformer_example");
  ros::NodeHandle nh("~");
  mrs_lib::Transformer tfr(nh);

  const std::string from = mrs_lib::LATLON_ORIGIN;
  const std::string to = mrs_lib::UTM_ORIGIN;
  geometry_msgs::Point pt;

  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    const auto tf_opt = tfr.getTransform(from, to);
    if (tf_opt.has_value())
    {
      const auto tf = tf_opt.value();
      std::cout << "Was looking for transformation from \"" << from << "\" to \"" << to << "\".\n";
      std::cout << "Received transformation from \"" << mrs_lib::Transformer::frame_from(tf) << "\" to \"" << mrs_lib::Transformer::frame_to(tf) << "\":\n";
      std::cout << tf << std::endl;
      const auto tfd_opt = tfr.transform(pt, tf);
      // don't forget to check whether the returned value is valid
      if (tfd_opt.has_value())
        std::cout << tfd_opt.value() << std::endl;
      break;
    }
  }

  return 0;
}
