/**  \file vector_converter.hpp
     \brief Implements the convertTo() and convertFrom() functions for conversion between different vector representations (Eigen, OpenCV, tf2 etc.).
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#ifndef VECTOR_CONVERTER_HPP
#define VECTOR_CONVERTER_HPP

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
    
    GENERATE_HAS_MEMBER_FUNC(x, double)
    GENERATE_HAS_MEMBER(x, double)

    template<class T> using _has_squarebracket_operator_chk = decltype(std::declval<T &>()[0]);
    template<class T> constexpr bool has_squarebracket_operator_v = std::experimental::is_detected_convertible_v<double, _has_squarebracket_operator_chk, T>;

    template<class T> constexpr bool has_xyz_constructor_v = std::experimental::is_constructible_v<T, double, double, double>;
    
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
    std::enable_if_t<has_xyz_constructor_v<ret_t>, void> convertTo(ret_t& out, const double x, const double y, const double z)
    {
      out = {x, y, z};
    }

    // convertTo specialization for other types
    template <typename ret_t>
    std::enable_if_t<!has_xyz_constructor_v<ret_t> && has_xmem_v<ret_t>, void> convertTo(ret_t& out, const double x, const double y, const double z)
    {
      out.x = x;
      out.y = y;
      out.z = z;
    }

    template <typename ret_t>
    ret_t convertTo(const double x, const double y, const double z)
    {
      ret_t ret;
      convertTo(ret, x, y, z);
      return ret;
    }

  }  // namespace impl

}  // namespace mrs_lib

#endif // VECTOR_CONVERTER_HPP
