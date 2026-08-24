#ifndef MRS_LIB_LOGGER_HPP_
#define MRS_LIB_LOGGER_HPP_

#include <chrono>
#include <format>
#include <utility>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>

// Macro to expand the tokens further.
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MRS_LIB_GENERATE_FOR_ALL_LEVELS_EXPAND(...) __VA_ARGS__

// Macro to generate methods of the logger without repeating them manually for all levels.
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MRS_LIB_GENERATE_FOR_ALL_LEVELS(MACRO)                                                                                                                 \
  MRS_LIB_GENERATE_FOR_ALL_LEVELS_EXPAND(MACRO(debug, DEBUG))                                                                                                  \
  MRS_LIB_GENERATE_FOR_ALL_LEVELS_EXPAND(MACRO(info, INFO))                                                                                                    \
  MRS_LIB_GENERATE_FOR_ALL_LEVELS_EXPAND(MACRO(warning, WARN))                                                                                                 \
  MRS_LIB_GENERATE_FOR_ALL_LEVELS_EXPAND(MACRO(error, ERROR))                                                                                                  \
  MRS_LIB_GENERATE_FOR_ALL_LEVELS_EXPAND(MACRO(fatal, FATAL))                                                                                                  \
  static_assert(true)

// Hide the lambda from Doxygen, because it breaks Sphinx rendering.
#define MRS_LIB_LOGGER_UNIQUE_TAG [] {}

namespace mrs_lib
{

  /**
   * @brief Wrapper for rclcpp::Logger with formatting using std::format.
   *
   * This wrapper can be used to wrap rclcpp::Logger and call log functions
   * without using the rclcpp macros.
   *
   * The log methods use std::format to format the messages. This allows adding
   * formatting for custom types by specializing the std::formatter struct.
   */
  class Logger
  {
    using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeLoggingInterface, rclcpp::node_interfaces::NodeClockInterface>;

  public:
    using Level = rclcpp::Logger::Level;

    /**
     * @brief Construct from node like object with the required interfaces.
     */
    explicit Logger(NodeInterfaces node) : node_interfaces_(std::move(node))
    {
    }

    /**
     * @brief Set output level of the stored logger.
     *
     * This affects the wrapped rclcpp::Logger, so it will for all calls using
     * that logger, not only using this wrapper.
     */
    void set_level(Level level)
    {
      node_interfaces_.get_node_logging_interface()->get_logger().set_level(level);
    }

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MRS_LIB_GEN_LOG(level, LEVEL)                                                                                                                          \
  /** @brief Log at the level specified in the name. */                                                                                                        \
  /** @param fmt Format string. */                                                                                                                             \
  /** @param args Arguments to format. */                                                                                                                      \
  template <typename... Args>                                                                                                                                  \
  void level(std::format_string<Args...> fmt, Args&&... args)                                                                                                  \
  {                                                                                                                                                            \
    RCLCPP_##LEVEL(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());                                                                 \
  }

    MRS_LIB_GENERATE_FOR_ALL_LEVELS(MRS_LIB_GEN_LOG);
#undef MRS_LIB_GEN_LOG

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MRS_LIB_GEN_LOG_ONCE(level, LEVEL)                                                                                                                     \
  /** @brief Log once at the level specified in the name. */                                                                                                   \
  /** @param fmt Format string. */                                                                                                                             \
  /** @param args Arguments to format. */                                                                                                                      \
  /** Only the first pass at each call site will generate the output, the rest will be ignored. */                                                             \
  template <auto LocTag = MRS_LIB_LOGGER_UNIQUE_TAG, typename... Args>                                                                                         \
  void level##_once(std::format_string<Args...> fmt, Args&&... args)                                                                                           \
  {                                                                                                                                                            \
    RCLCPP_##LEVEL##_ONCE(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());                                                          \
  }

    MRS_LIB_GENERATE_FOR_ALL_LEVELS(MRS_LIB_GEN_LOG_ONCE);
#undef MRS_LIB_GEN_LOG_ONCE

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MRS_LIB_GEN_LOG_THROTTLE(level, LEVEL)                                                                                                                 \
  /** @brief Log at the level specified in the name with a maximum rate. */                                                                                    \
  /** @param duration Minimum time between two consecutive logs from this call site. */                                                                        \
  /** @param fmt Format string. */                                                                                                                             \
  /** @param args Arguments to format. */                                                                                                                      \
  template <auto LocTag = MRS_LIB_LOGGER_UNIQUE_TAG, typename... Args>                                                                                         \
  void level##_throttle(std::chrono::milliseconds duration, std::format_string<Args...> fmt, Args&&... args)                                                   \
  {                                                                                                                                                            \
    RCLCPP_##LEVEL##_THROTTLE(get_logger(), get_clock(), duration.count(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());                       \
  }

    MRS_LIB_GENERATE_FOR_ALL_LEVELS(MRS_LIB_GEN_LOG_THROTTLE);
#undef MRS_LIB_GEN_LOG_THROTTLE

  private:
    rclcpp::Logger get_logger()
    {
      return node_interfaces_.get_node_logging_interface()->get_logger();
    }

    rclcpp::Clock& get_clock()
    {
      return *node_interfaces_.get_node_clock_interface()->get_clock();
    }

    NodeInterfaces node_interfaces_;
  };

} // namespace mrs_lib

#undef MRS_LIB_LOGGER_UNIQUE_TAG
#undef MRS_LIB_GENERATE_FOR_ALL_LEVELS
#undef MRS_LIB_GENERATE_FOR_ALL_LEVELS_EXPAND

#endif // MRS_LIB_LOGGER_HPP_
