#include <mrs_lib/transformer.h>

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
      std::cout << "Transformation from " << from << " to " << to << ":\n";
      std::cout << tf_opt.value() << std::endl;
      break;
    }
  }
}
