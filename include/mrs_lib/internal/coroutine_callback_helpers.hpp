#ifndef MRS_LIB_INTERNAL_COROUTINE_CALLBACK_HELPERS_HPP_
#define MRS_LIB_INTERNAL_COROUTINE_CALLBACK_HELPERS_HPP_


#include <rclcpp/callback_group.hpp>


namespace mrs_lib::internal
{

  /**
   * @brief Check that callback group type is compatible with coroutine callbacks.
   *
   * To simplify the behavior of coroutine callbacks, they currently only
   * allowed when using reentrant callback group.
   *
   * @param callback_group Callback group to check. This can be nullptr.
   *
   * @return True if the callback group is coroutine compatible, false otherwise.
   */
  inline bool is_callback_group_coro_compatible(const std::shared_ptr<rclcpp::CallbackGroup>& callback_group)
  {
    return callback_group->type() == rclcpp::CallbackGroupType::Reentrant;
  }

  /**
   * @brief Check that callback group type is compatible with coroutine callbacks. Throws if it is not.
   *
   * To simplify the behavior of coroutine callbacks, they currently only
   * allowed when using reentrant callback group.
   *
   * @param callback_group Callback group to check. This can be nullptr.
   *
   * @throws std::logic_error if the callback group is not compatible with coroutine callbacks.
   */
  inline void require_callback_group_coro_compatible(const std::shared_ptr<rclcpp::CallbackGroup>& callback_group)
  {
    if (callback_group == nullptr || callback_group->type() != rclcpp::CallbackGroupType::Reentrant)
    {
      std::string msg = "Coroutine callbacks must be used with reentrant callback group.";
      RCLCPP_ERROR(rclcpp::get_logger("mrs_lib"), "%s", msg.c_str());
      throw std::logic_error(msg);
    }
  }

} // namespace mrs_lib::internal

#endif // MRS_LIB_INTERNAL_COROUTINE_CALLBACK_HELPERS_HPP_
