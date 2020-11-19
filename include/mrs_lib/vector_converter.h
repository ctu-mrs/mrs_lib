#include <Eigen/Dense>
#include <tuple>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <experimental/type_traits>

namespace mrs_lib
{
  namespace impl
  {
    using unw_t = std::tuple<double, double, double>;

#define GENERATE_HAS_MEMBER_FUNC(func, rettype)                                \
template<class T> using _has_##func_chk =                                      \
      decltype(std::declval<T &>().func());                                    \
template<class T> constexpr bool has_##func##fun_v =                           \
      std::experimental::is_detected_convertible_v<rettype, _has_##func_chk, T>;

#define GENERATE_HAS_MEMBER(memb, type)                                        \
template<class T> using _has_##memb_chk =                                      \
      decltype(std::declval<T &>().memb);                                      \
template<class T> constexpr bool has_##memb##mem_v =                           \
      std::experimental::is_detected_convertible_v<type, _has_##memb_chk, T>;

    GENERATE_HAS_MEMBER_FUNC(x, double);
    GENERATE_HAS_MEMBER(x, double);

    template <typename in_t>
    std::enable_if_t<has_xfun_v<in_t>, unw_t> convertFrom(const in_t& in)
    {
      return {in.x(), in.y(), in.z()};
    }

    template <typename in_t>
    std::enable_if_t<has_xmem_v<in_t>, unw_t> convertFrom(const in_t& in)
    {
      return {in.x, in.y, in.z};
    }

    template <typename in_t>
    std::enable_if_t<!has_xfun_v<in_t> && !has_xmem_v<in_t>, unw_t> convertFrom(const in_t& in)
    {
      return {in[0], in[1], in[2]};
    }

    template <typename ret_t>
    ret_t convertTo(const double x, const double y, const double z)
    {
      return ret_t {x, y, z};
    }

    template <>
    geometry_msgs::Vector3 convertTo<geometry_msgs::Vector3>(const double x, const double y, const double z)
    {
      geometry_msgs::Vector3 ret;
      ret.x = x;
      ret.y = y;
      ret.z = z;
      return ret;
    }

  }

  template <typename ret_t, typename in_t>
  ret_t convert(const in_t& in)
  {
    const auto [x, y, z] = impl::convertFrom(in);
    return impl::convertTo<ret_t>(x, y, z);
  }

}
