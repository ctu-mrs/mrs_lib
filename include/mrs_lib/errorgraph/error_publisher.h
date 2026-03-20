#pragma once

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
       * \note To make sure that the published messages are propagated through ROS to any subscribers,
       * the method waits 1s after publishing before calling rclcpp::shutdown().
       */
      void flushAndShutdown();

      /*!
       * \brief Add a custom error to the list of aggregated errors to be published in the next period.
       * This method uses the `id` to distinguish between different error types and avoid error duplication.
       * If an element with the same `id` is found in the list of currently aggregated errors, it will be replaced
       * by the new one provided. Otherwise, a new error will be added to the list.
       *
       * \param id               The unique identification number of this error type for this ErrorPublisher.
       * \param description      A short and succinct description of the error.
       */
      void addGeneralError(const error_id_t id, const std::string& description);

      /*!
       * \brief A convenience overload for custom enumeration types.
       * This overload just casts the `id` parameter to the `error_id_t` type so that you can call it with
       * your own enumeration type more easily without casting it yourself.
       *
       * \param id               The unique identification number of this error type for this ErrorPublisher.
       * \param description      A short and succinct description of the error.
       */
      template <typename enum_T>
        requires(std::is_enum_v<enum_T> && sizeof(std::underlying_type_t<enum_T>) <= sizeof(error_id_t))
      void addGeneralError(const enum_T id, const std::string& description)
      {
        addGeneralError(static_cast<error_id_t>(id), description);
      }

      /*!
       * \brief Add an error that only appears once.
       * This overload assumes that the error is unique, so it cannot be duplicate and doesn't need an identifier.
       * Useful for errors during initialization that lead to termination of the node anyways.
       *
       * \param description      A short and succinct description of the error.
       */
      void addOneshotError(const std::string& description);

      /*!
       * \brief Add a special error type `waiting_for_node`.
       * Use this whenever your node is blocked because it waits for some kind of input from another node, such as
       * a transformation, messages, a service, etc. This helps construct the Errorgraph and decide which errors
       * are the roots problems blocking the system. If you don't know the component of the node for the node_id,
       * use `main`.
       *
       * \param node_id          Identifier of the node and component that is being waited for. If the component is unknown, use `main`.
       */
      void addWaitingForNodeError(const node_id_t& node_id);

      /*!
       * \brief Add a special error type `waiting_for_topic`.
       * You should prioritize using the addWaitingForNodeError() method if you know the node that should publish
       * this topic to provide better information to the Errorgraph. Use this method if the node publishing
       * the topic is unknown or can change.
       *
       * \param topic_name       Full name of the topic that is being waited for.
       */
      void addWaitingForTopicError(const std::string& topic_name);

    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Clock::SharedPtr clock_;
      std::string node_name_;
      std::string component_name_;

      std::mutex errors_mtx_;
      std::vector<error_wrapper_t> errors_;

      rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

      rclcpp::Publisher<mrs_msgs::msg::ErrorgraphElement>::SharedPtr publisher_;

      std::unique_ptr<mrs_lib::MRSTimer> timer_publisher_;

      void publishErrors();
    };

  } // namespace errorgraph
} // namespace mrs_lib
