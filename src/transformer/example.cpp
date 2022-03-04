#include <mrs_lib/transformer.h>

// very rudimentary
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "transformer_example");
  ros::NodeHandle nh("~");
  mrs_lib::Transformer tfr(nh);

  const std::string from = "from";
  const std::string to = "to";
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
      break;
    }
  }
}
