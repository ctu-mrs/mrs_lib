#ifndef VECTOR_CONVERTER_H
#define VECTOR_CONVERTER_H

#include <tuple>

#include <impl/vector_converter.hpp>
namespace mrs_lib
{

  template <typename ret_t, typename in_t>
  ret_t convert(const in_t& in)
  {
    const auto [x, y, z] = impl::convertFrom(in);
    return impl::convertTo<ret_t>(x, y, z);
  }

}

#endif // VECTOR_CONVERTER_H
