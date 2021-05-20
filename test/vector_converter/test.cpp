#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#include <mrs_lib/vector_converter.h>
#pragma GCC diagnostic pop
#include <impl/vector_converter_types.h>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

using namespace std;

typedef ::testing::Types<VC_TYPE_LIST> VectorTypes;

/* SINGLE-TYPE TESTS //{ */

template <typename T>
class VectorConverterTest: public ::testing::Test { };
TYPED_TEST_CASE_P(VectorConverterTest);

TYPED_TEST_P(VectorConverterTest, ConvertsTo)
{
  [[maybe_unused]] const TypeParam vec = mrs_lib::impl::convertTo<TypeParam>(rand(), rand(), rand());;
}

TYPED_TEST_P(VectorConverterTest, ConvertsFrom)
{
  const TypeParam vec;
  [[maybe_unused]] const auto ret = mrs_lib::impl::convertFrom(vec);
}

TYPED_TEST_P(VectorConverterTest, ConvertsValid)
{
  const double xo = rand(), yo = rand(), zo = rand();
  const TypeParam vec = mrs_lib::impl::convertTo<TypeParam>(xo, yo, zo);;
  const auto [x, y, z] = mrs_lib::impl::convertFrom(vec);
  EXPECT_TRUE((x == xo && y == yo && z == zo) || (x == float(xo) && y == float(yo) && z == float(zo)));
}

REGISTER_TYPED_TEST_CASE_P(VectorConverterTest,
    ConvertsTo,
    ConvertsFrom,
    ConvertsValid
);

INSTANTIATE_TYPED_TEST_CASE_P(VectorTypesInstantiation, VectorConverterTest, VectorTypes);

//}

/* FROM-TO TYPE CONVERSION TESTS //{ */

// This is a single testing function for converting from from_t to to_t
template <typename from_t, typename to_t>
bool testConv()
{                                                                   
  const double xo = rand(), yo = rand(), zo = rand();               
  const from_t from = mrs_lib::impl::convertTo<from_t>(xo, yo, zo); 
  const to_t to = mrs_lib::convert<to_t>(from);                     
  const auto [x, y, z] = mrs_lib::impl::convertFrom(to);            
  return                                                            
      (x == xo && y == yo && z == zo)                               
   || (x == float(xo) && y == float(yo) && z == float(zo));         
}

/* TMP BLACK MAGIC, ALSO NO TOUCHING! //{ */

template <std::size_t I, typename tuple_t>
void test_Ith_tuple()
{
  using type_list = tuple_t;
  constexpr auto n_types = std::tuple_size<type_list>::value;
  using from_t = typename std::tuple_element<I/n_types, type_list>::type;
  using to_t = typename std::tuple_element<I%n_types, type_list>::type;
  const auto result = testConv<from_t, to_t>();
  EXPECT_TRUE(result) << "Conversion from type " << typeid(from_t).name() << " to type " << typeid(to_t).name() << " failed.";
  std::cout << "Testing conversion from type " << typeid(from_t).name() << " to type " << typeid(to_t).name() << "." << std::endl;
}

template <typename tuple_t, std::size_t ... Is>
void static_for_helper(std::index_sequence<Is...>)
{
  (test_Ith_tuple<Is, tuple_t>(),
   ...);
}

template <std::size_t N, typename tuple_t>
void static_for()
{
  static_for_helper<tuple_t>(std::make_index_sequence<N>{});
}

//}

// This test tests all combinations of conversions between the various types, defined in "impl/vector_converter_types.h"
TEST(TESTSuite, ConvertsBetween)
{
  std::vector<std::function<bool()>> conv_tests;
  // Define test functions for converting between the different types using a cartesian product of the type list with itself
  /* DEFINE_TESTS_CARTESIAN((VC_TYPE_LIST), (VC_TYPE_LIST)) */
  using type_list = std::tuple<VC_TYPE_LIST>;
  const type_list tlist_inst;
  constexpr auto n_types = std::tuple_size<type_list>::value;
  // Run all the conversion test functions, checking the return value
  static_for<n_types*n_types, type_list>();
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

