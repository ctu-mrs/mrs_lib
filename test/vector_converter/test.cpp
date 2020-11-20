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

/* HELPER MACROS //{ */

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
#define MAKE_UNIQUE(x) CONCATENATE(x, __COUNTER__)

//}

// This macro defines a single testing function for converting from FROM_TYPE to TO_TYPE and pushes it to conv_tests
#define DEFINE_TEST(FROM_TYPE, TO_TYPE)                             \
conv_tests.push_back([] ()                                          \
{                                                                   \
  using from_t = FROM_TYPE;                                         \
  using to_t = TO_TYPE;                                             \
  const double xo = rand(), yo = rand(), zo = rand();               \
  const from_t from = mrs_lib::impl::convertTo<from_t>(xo, yo, zo); \
  const to_t to = mrs_lib::convert<to_t>(from);                     \
  const auto [x, y, z] = mrs_lib::impl::convertFrom(to);            \
  return                                                            \
      (x == xo && y == yo && z == zo)                               \
   || (x == float(xo) && y == float(yo) && z == float(zo));         \
});

/* PREPROCESSOR BLACK MAGIC, NO TOUCHY TOUCHY! //{ */

// Boost preprocessor lib for cartesian product of type lists
#include <boost/preprocessor/seq/for_each_product.hpp>
#include <boost/preprocessor/seq/to_tuple.hpp>
#include <boost/preprocessor/tuple/to_seq.hpp>

#define EVAL(...) __VA_ARGS__
#define DEFINE_TESTS_SEMI_DELIM(R,SEQ_X) EVAL(DEFINE_TEST BOOST_PP_SEQ_TO_TUPLE(SEQ_X));
#define DEFINE_TESTS_CARTESIAN(TUP_A,TUP_B) \
   BOOST_PP_SEQ_FOR_EACH_PRODUCT \
   ( DEFINE_TESTS_SEMI_DELIM, \
     (BOOST_PP_TUPLE_TO_SEQ(TUP_A)) \
     (BOOST_PP_TUPLE_TO_SEQ(TUP_B)) \
   )

//}

/* TMP BLACK MAGIC, ALSO NO TOUCHING! //{ */

template <std::size_t I, typename tuple_t, typename F>
void test_Ith_tuple(F fs)
{
  using type_list = tuple_t;
  constexpr auto n_types = std::tuple_size<type_list>::value;
  using from_t = std::tuple_element<I/n_types, type_list>;
  using to_t = std::tuple_element<I%n_types, type_list>;
  EXPECT_TRUE(fs.at(I)()) << "Conversion from type " << typeid(from_t).name() << " to type " << typeid(to_t).name() << " failed.";
}

template <typename tuple_t, std::size_t ... Is, typename F>
void static_for_helper(std::index_sequence<Is...>, F fs)
{
  (test_Ith_tuple<Is, tuple_t>(fs),
   ...);
}

template <std::size_t N, typename tuple_t, typename F>
void static_for(F f)
{
  static_for_helper<tuple_t>(std::make_index_sequence<N>{}, f);
}

//}

// This test tests all combinations of conversions between the various types, defined in "impl/vector_converter_types.h"
TEST(TESTSuite, ConvertsBetween)
{
  std::vector<std::function<bool()>> conv_tests;
  // Define test functions for converting between the different types using a cartesian product of the type list with itself
  DEFINE_TESTS_CARTESIAN((VC_TYPE_LIST), (VC_TYPE_LIST))
  using type_list = std::tuple<VC_TYPE_LIST>;
  const type_list tlist_inst;
  constexpr auto n_types = std::tuple_size<type_list>::value;
  // Run all the conversion test functions, checking the return value
  static_for<n_types, type_list>(conv_tests);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

