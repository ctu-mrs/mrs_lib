#include "mrs_lib/logger.hpp"

#include <cstddef>

#include <iostream>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <string_view>
#include <thread>

int main(int argc, char** argv)
{

  auto args = std::span(argv, argc);
  if (args.size() != 2)
  {
    std::cerr << "Expected level argument.\n";
    return 1;
  }

  std::string_view level_arg = args[1];
  auto level = mrs_lib::Logger::Level::Debug;
  if (level_arg == "DEBUG")
  {
    level = mrs_lib::Logger::Level::Debug;
  } else if (level_arg == "INFO")
  {
    level = mrs_lib::Logger::Level::Info;
  } else if (level_arg == "WARN")
  {
    level = mrs_lib::Logger::Level::Warn;
  } else if (level_arg == "ERROR")
  {
    level = mrs_lib::Logger::Level::Error;
  } else if (level_arg == "FATAL")
  {
    level = mrs_lib::Logger::Level::Fatal;
  } else
  {
    std::cerr << "Unhandled level argument: '" << level_arg << "'\n";
    return 1;
  }


  rclcpp::init(argc, argv);

  // DOCS: BEGIN EXAMPLE
  using namespace std::chrono_literals;

  auto node = std::make_shared<rclcpp::Node>("logger_test_node");
  auto logger = mrs_lib::Logger(*node);
  logger.set_level(level);

  {
    logger.debug("Hi");
    logger.debug("Hello {}", "world");

    for (size_t i = 0; i < 2; ++i)
    {
      logger.debug_once("Only Once");
      logger.debug_once("Also this is only once");
    }

    for (size_t i = 0; i < 4; ++i)
    {
      logger.debug_throttle(100ms, "Throttled a: {}", i);
      logger.debug_throttle(100ms, "Throttled b: {}", i);
      std::this_thread::sleep_for(40ms);
    }
  }
  // DOCS: END EXAMPLE

  {
    logger.info("Hi");
    logger.info("Hello {}", "world");

    for (size_t i = 0; i < 2; ++i)
    {
      logger.info_once("Only Once");
      logger.info_once("Also this is only once");
    }

    for (size_t i = 0; i < 4; ++i)
    {
      logger.info_throttle(100ms, "Throttled a: {}", i);
      logger.info_throttle(100ms, "Throttled b: {}", i);
      std::this_thread::sleep_for(40ms);
    }
  }

  {
    logger.warning("Hi");
    logger.warning("Hello {}", "world");

    for (size_t i = 0; i < 2; ++i)
    {
      logger.warning_once("Only Once");
      logger.warning_once("Also this is only once");
    }

    for (size_t i = 0; i < 4; ++i)
    {
      logger.warning_throttle(100ms, "Throttled a: {}", i);
      logger.warning_throttle(100ms, "Throttled b: {}", i);
      std::this_thread::sleep_for(40ms);
    }
  }

  {
    logger.error("Hi");
    logger.error("Hello {}", "world");

    for (size_t i = 0; i < 2; ++i)
    {
      logger.error_once("Only Once");
      logger.error_once("Also this is only once");
    }

    for (size_t i = 0; i < 4; ++i)
    {
      logger.error_throttle(100ms, "Throttled a: {}", i);
      logger.error_throttle(100ms, "Throttled b: {}", i);
      std::this_thread::sleep_for(40ms);
    }
  }

  {
    logger.fatal("Hi");
    logger.fatal("Hello {}", "world");

    for (size_t i = 0; i < 2; ++i)
    {
      logger.fatal_once("Only Once");
      logger.fatal_once("Also this is only once");
    }

    for (size_t i = 0; i < 4; ++i)
    {
      logger.fatal_throttle(100ms, "Throttled a: {}", i);
      logger.fatal_throttle(100ms, "Throttled b: {}", i);
      std::this_thread::sleep_for(40ms);
    }
  }

  rclcpp::shutdown();
}
