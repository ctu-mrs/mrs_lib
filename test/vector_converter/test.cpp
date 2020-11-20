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

// This macro defines a test for converting from FROM_TYPE to TO_TYPE
#define DEFINE_TEST(FROM_TYPE, TO_TYPE)                             \
TEST(VectorConverterTest, MAKE_UNIQUE(ConvertsBetween) )            \
{                                                                   \
  using from_t = FROM_TYPE;                                         \
  using to_t = TO_TYPE;                                             \
  const double xo = rand(), yo = rand(), zo = rand();               \
  const from_t from = mrs_lib::impl::convertTo<from_t>(xo, yo, zo); \
  const to_t to = mrs_lib::convert<to_t>(from);                     \
  const auto [x, y, z] = mrs_lib::impl::convertFrom(to);            \
  EXPECT_TRUE(                                                      \
      (x == xo && y == yo && z == zo)                               \
   || (x == float(xo) && y == float(yo) && z == float(zo)));        \
}

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

// Define tests for converting between the different types using a cartesian product of the type list with itself
DEFINE_TESTS_CARTESIAN((VC_TYPE_LIST), (VC_TYPE_LIST))

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

