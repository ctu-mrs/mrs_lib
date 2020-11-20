#ifndef VECTOR_CONVERTER_HPP
#define VECTOR_CONVERTER_HPP

#include <mrs_lib/vector_converter.h>
#include <experimental/type_traits>

namespace mrs_lib
{
  namespace impl
  {
    using unw_t = std::tuple<double, double, double>;

    /* SFINAE magic - only for black wizards! //{ */
    
    #define GENERATE_HAS_MEMBER_FUNC(func, rettype)                                \
    template<class T> using _has_##func##fun_chk =                                 \
          decltype(std::declval<T &>().func());                                    \
    template<class T> constexpr bool has_##func##fun_v =                           \
          std::experimental::is_detected_convertible_v<rettype, _has_##func##fun_chk, T>;
    
    #define GENERATE_HAS_MEMBER(memb, type)                                        \
    template<class T> using _has_##memb##mem_chk =                                 \
          decltype(std::declval<T &>().memb);                                      \
    template<class T> constexpr bool has_##memb##mem_v =                           \
          std::experimental::is_detected_convertible_v<type, _has_##memb##mem_chk, T>;
    
    GENERATE_HAS_MEMBER_FUNC(x, double);
    GENERATE_HAS_MEMBER(x, double);

    template<class T> using _has_squarebracket_operator_chk = decltype(std::declval<T &>()[0]);
    template<class T> constexpr bool has_squarebracket_operator_v = std::experimental::is_detected_convertible_v<double, _has_squarebracket_operator_chk, T>;

    template<class T> using _has_xyz_constructor_chk = decltype(T{std::declval<double>(), std::declval<double>(), std::declval<double>()});
    template<class T> constexpr bool has_xyz_constructor_v = std::experimental::is_detected_v<_has_xyz_constructor_chk, T>;
    
    //}

    // convertFrom specialization for Eigen types
    template <typename in_t>
    std::enable_if_t<has_xfun_v<in_t>, unw_t> convertFrom(const in_t& in)
    {
      return {in.x(), in.y(), in.z()};
    }

    // convertFrom specialization for plain member types
    template <typename in_t>
    std::enable_if_t<has_xmem_v<in_t>, unw_t> convertFrom(const in_t& in)
    {
      return {in.x, in.y, in.z};
    }

    // convertFrom specialization for OpenCV vectors
    template <typename in_t>
    std::enable_if_t<has_squarebracket_operator_v<in_t> && !has_xfun_v<in_t>, unw_t> convertFrom(const in_t& in)
    {
      return {in[0], in[1], in[2]};
    }

    // convertTo specialization for types with a constructor that takes three doubles
    template <typename ret_t>
    std::enable_if_t<has_xyz_constructor_v<ret_t>, ret_t> convertTo(const double x, const double y, const double z)
    {
      return ret_t {x, y, z};
    }

    // convertTo specialization for other types
    template <typename ret_t>
    std::enable_if_t<!has_xyz_constructor_v<ret_t>, ret_t> convertTo(const double x, const double y, const double z)
    {
      ret_t ret;
      ret.x = x;
      ret.y = y;
      ret.z = z;
      return ret;
    }

  }  // namespace impl

}  // namespace mrs_lib

#endif // VECTOR_CONVERTER_HPP
