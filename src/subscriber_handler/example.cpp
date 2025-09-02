// clang: MatousFormat
/**  \file
     \brief Example file for the SubscribeHandler convenience class
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib subscribe_handler_example`.

     See \ref subscriber_handler/example.cpp.
 */

/**  \example "subscriber_handler/example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib subscribe_handler_example`.
 */

// Include the SubscribeHandler header
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/impl/subscriber_handler.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>  // Add this for std::bind

void timeout_callback(rclcpp::Node::SharedPtr node, const std::string& topic_name, const rclcpp::Time& last_msg)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("timeout_callback"), "Have not received a message on topic '" << topic_name << "' for " << (node->now() - last_msg).seconds() << " seconds");
}

void message_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("message_callback"), "Received: '" << msg->data << "'.");
}

class SubObject
{
  public:
    void callback_method(const std_msgs::msg::String::ConstSharedPtr msg)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("callback method"), "Object received: '" << msg->data << "'.");
    }

    void timeout_method(rclcpp::Node::SharedPtr node, const std::string& topic_name, const rclcpp::Time& last_msg)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("timer method"), "Object has not received a message on topic '" << topic_name << "' for " << (node->now() - last_msg).seconds() << " seconds");
    }
} sub_obj;

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name("subscribe_handler_example");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(node_name);

  /* name of the topic to be handled */
  const std::string topic_name = "test_topic";
  /* after this duration without receiving messages on the handled topic, the timeout_callback will be called */
  const rclcpp::Duration no_message_timeout = rclcpp::Duration(1.0, 0);

  RCLCPP_INFO(node->get_logger(), "[%s]: Creating SubscriberHandlers.", node_name.c_str());

  mrs_lib::SubscriberHandlerOptions shopts(node);
  shopts.node_name = node_name;
  shopts.no_message_timeout = no_message_timeout;

  /* This is how a new SubscriberHandler object is initialized. */ 
  mrs_lib::SubscriberHandler<std_msgs::msg::String> handler(
            shopts,
            topic_name,
            std::bind(timeout_callback, node, std::placeholders::_1, std::placeholders::_2),
            message_callback
            );

  /* A variation of the factory method for easier use with objects also exists. */ 
  mrs_lib::construct_object(
            handler,
            shopts,
            topic_name,
            no_message_timeout,
            std::bind(&SubObject::timeout_method, &sub_obj, node, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SubObject::callback_method, &sub_obj, std::placeholders::_1)
            );

  /* Type of the message may be accessed by C++11 decltype in case of need */ 
  using message_type = mrs_lib::message_type<decltype(handler)>;
  rclcpp::Publisher<message_type>::SharedPtr publisher_ = node->create_publisher<message_type>(topic_name, 5);

  /* Now let's just spin to process calbacks until the user decides to stop the program. */ 
  rclcpp::Rate rate(10);
  while (true)
  {
    message_type msg;
    msg.data = "asdf";
    publisher_->publish(msg);
    RCLCPP_INFO(node->get_logger(), "[%s]: Spinning", node->get_name());
    rclcpp::spin_some(node);
    rate.sleep();
  }
}
