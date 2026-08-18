#pragma once

#include <optional>
#include <set>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/errorgraph/node_id.h>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_lib/timer_handler.h>

namespace mrs_lib
{
  namespace errorgraph
  {

    /**
     * \brief A helper class for aggregating and publishing errors to the Errorgraph.
     * Report errors preventing your node from functioning properly using the respective methods.
     * These are aggregated and periodically published by this class. After publishing, the list
     * of errors is cleared, so take care when setting the `publish_period`.
     *
     * Every add*Error() method also accepts an optional trailing `component_override`, letting several
     * logical components that share one instance (e.g. a manager and its plugins) be told apart in the
     * published Errorgraph without needing one ErrorPublisher per component. See addGeneralError()'s
     * `component_override` docs for the constraints on its value.
     */
    class ErrorPublisher
    {
    public:
      /*!
       * \brief Type of the ID used to avoid duplication of errors when using addGeneralError.
       */
      using error_id_t = uint16_t;

    private:
      struct error_wrapper_t
      {
        std::optional<error_id_t> id;
        std::string component; ///< component_override.value_or(component_name_), resolved once at add-time.
        mrs_msgs::msg::ErrorgraphError msg;
      };

    public:
      /*!
       * \brief The main constructor.
       *
       * \param node             The ROS2 node used for publisher advertisement and timer registration.
       * \param clock            The clock used for timestamping the published error messages.
       * \param node_name        Name of the ROS node used for filling out the node_id in the published error messages.
       * \param component_name   Name of the component used for filling out the node_id in the published error messages.
       * \param publish_period   How often will the aggregated errors be published.
       */
      ErrorPublisher(const rclcpp::Node::SharedPtr node, const rclcpp::Clock::SharedPtr clock, const std::string& node_name, const std::string& component_name,
                     const rclcpp::Rate& publish_period = rclcpp::Rate(1.0));

      /*!
       * \brief Publishes all aggregated errors and calls rclcpp::shutdown().
       *
       * Heartbeat-only elements (components with nothing currently pending) are suppressed on this
       * path, so only components with something urgent to say are published.
       *
       * \note To make sure that the published messages are propagated through ROS to any subscribers,
       * the method waits 1s after publishing before calling rclcpp::shutdown().
       * \warning Calls `exit(1)`, terminating the whole process, not just this ErrorPublisher. If this
       * instance is shared across multiple components (see class docs), any one of them calling this
       * over its own fatal error takes the entire node down, including the other components.
       */
      void flushAndShutdown();

      /*!
       * \brief Add a custom error to the list of aggregated errors to be published in the next period.
       * This method uses the `id`, scoped to the resolved component (`component_override` if given,
       * otherwise `component_name`), to distinguish between different error types and avoid error
       * duplication. If an element with the same `id` under the same resolved component is found in
       * the list of currently aggregated errors, it will be replaced by the new one provided. Otherwise,
       * a new error will be added to the list. The same `id` may be reused across different components
       * (default or overridden) without colliding — they are tracked independently.
       *
       * \param id               The unique identification number of this error type, scoped to the resolved component.
       * \param description      A short and succinct description of the error.
       * \param component_override  Reports this error under a different component than `component_name`.
       *                             Must be a small, static, per-instance value (e.g. a fixed plugin name), never a
       *                             dynamically generated string (topic name, loop counter, ...): every distinct value
       *                             is remembered for the instance's lifetime and gets its own published element per period.
       */
      void addGeneralError(const error_id_t id, const std::string& description, const std::optional<std::string>& component_override = std::nullopt);

      /*!
       * \brief A convenience overload for custom enumeration types.
       * This overload just casts the `id` parameter to the `error_id_t` type so that you can call it with
       * your own enumeration type more easily without casting it yourself.
       *
       * \param id               The unique identification number of this error type for this ErrorPublisher.
       * \param description      A short and succinct description of the error.
       * \param component_override  See addGeneralError()'s docs.
       */
      template <typename enum_T>
        requires(std::is_enum_v<enum_T> && sizeof(std::underlying_type_t<enum_T>) <= sizeof(error_id_t))
      void addGeneralError(const enum_T id, const std::string& description, const std::optional<std::string>& component_override = std::nullopt)
      {
        addGeneralError(static_cast<error_id_t>(id), description, component_override);
      }

      /*!
       * \brief Add an error that only appears once.
       * This overload assumes that the error is unique, so it cannot be duplicate and doesn't need an identifier.
       * Useful for errors during initialization that lead to termination of the node anyways.
       *
       * \param description      A short and succinct description of the error.
       * \param component_override  See addGeneralError()'s docs.
       */
      void addOneshotError(const std::string& description, const std::optional<std::string>& component_override = std::nullopt);

      /*!
       * \brief Add a special error type `waiting_for_node`.
       * Use this whenever your node is blocked because it waits for some kind of input from another node, such as
       * a transformation, messages, a service, etc. This helps construct the Errorgraph and decide which errors
       * are the roots problems blocking the system. If you don't know the component of the node for the node_id,
       * use `main`.
       *
       * \param node_id          Identifier of the node and component that is being waited for. If the component is unknown, use `main`.
       * \param component_override  See addGeneralError()'s docs.
       */
      void addWaitingForNodeError(const node_id_t& node_id, const std::optional<std::string>& component_override = std::nullopt);

      /*!
       * \brief Add a special error type `waiting_for_topic`.
       * You should prioritize using the addWaitingForNodeError() method if you know the node that should publish
       * this topic to provide better information to the Errorgraph. Use this method if the node publishing
       * the topic is unknown or can change.
       *
       * \param topic_name       Full name of the topic that is being waited for.
       * \param component_override  See addGeneralError()'s docs.
       */
      void addWaitingForTopicError(const std::string& topic_name, const std::optional<std::string>& component_override = std::nullopt);

      /*!
       * \brief Add a special error type `waiting_for_topic`.
       * If the expected publisher of the topic is known, use this method to provide better information to the Errorgraph. If the expected publisher is unknown
       * or can change, use the overload without the `expected_publisher` parameter instead.
       *
       * \param topic_name       Full name of the topic that is being waited for.
       * \param expected_publisher  Identifier of the node and component that is expected to publish this topic.
       * \param component_override  See addGeneralError()'s docs.
       */
      void addWaitingForTopicError(const std::string& topic_name, const node_id_t& expected_publisher,
                                   const std::optional<std::string>& component_override = std::nullopt);


    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Clock::SharedPtr clock_;
      std::string node_name_;
      std::string component_name_;

      std::mutex errors_mtx_;
      std::vector<error_wrapper_t> errors_;
      /// All components ever seen; pre-seeds publishErrors() so a quiet component still gets a heartbeat.
      /// Guarded by errors_mtx_, same as errors_.
      std::set<std::string> known_components_;

      rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

      rclcpp::Publisher<mrs_msgs::msg::ErrorgraphElement>::SharedPtr publisher_;

      std::unique_ptr<mrs_lib::MRSTimer> timer_publisher_;

      /*!
       * \brief Groups the aggregated errors by component and publishes one ErrorgraphElement per component.
       *
       * \param skip_empty_heartbeats  If true, skip components with no pending errors (no heartbeat).
       *                                Used on the shutdown path, where heartbeats are pointless.
       */
      void publishErrors(bool skip_empty_heartbeats = false);
    };

  } // namespace errorgraph
} // namespace mrs_lib
