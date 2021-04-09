// clang: MatousFormat
/**  \file vector_converter.h
     \brief Defines the convert() function for conversion between different vector representations (Eigen, OpenCV, tf2 etc.).
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#ifndef VECTOR_CONVERTER_H
#define VECTOR_CONVERTER_H

#include <tuple>

#include <impl/vector_converter.hpp>

namespace mrs_lib
{

  /*!
   * \brief Converts between different vector representations.
   *
   * Usage (for full example, see the file vector_converter/example.cpp):
   *
   * `auto out = mrs_lib::convert<to_type>(in);`
   *
   * Supported types by default are: `Eigen::Vector3d`, `cv::Vec3d`, `geometry_msgs::Vector3`.
   * If you want to use this with a new type, it should work automagically if it provides a reasonable API.
   * If it doesn't work by default, you just need to implement the respective convertTo() and convertFrom() functions for that type (see the file impl/vector_converter.hpp and the example file vector_converter/example.cpp).
   *
   * \param in  The input vector to be converted to the \p ret_t type.
   *
   * \tparam ret_t Type of the return vector. You need to specify this when calling the function.
   * \tparam in_t  Type of the input vector. This parameter is deduced by the compiler and doesn't need to be specified in usual cases.
   *
   */
  template <typename ret_t, typename in_t>
  ret_t convert(const in_t& in)
  {
    const auto [x, y, z] = impl::convertFrom(in);
    ret_t ret;
    impl::convertTo(ret, x, y, z);
    return ret;
  }

}

#endif // VECTOR_CONVERTER_H
