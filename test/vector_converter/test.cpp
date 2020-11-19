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

/* template <typename T> */
/* class VectorConverterTestCP: public ::testing::Test { }; */
/* TYPED_TEST_CASE_P(VectorConverterTestCP); */

/* TYPED_TEST_P(VectorConverterTestCP, ConvertsBetween) */
/* { */
/*   using from_t = typename TypeParam::from_type; */
/*   using to_t = typename TypeParam::to_type; */
/*   const double xo = rand(), yo = rand(), zo = rand(); */
/*   const from_t vec = mrs_lib::impl::convertTo<from_t>(xo, yo, zo);; */
/*   const to_t vec2 = mrs_lib::convert<to_t>(vec);; */
/*   const auto [x, y, z] = mrs_lib::impl::convertFrom(vec2); */
/*   EXPECT_TRUE((x == xo && y == yo && z == zo) || (x == float(xo) && y == float(yo) && z == float(zo))); */
/* } */

/* REGISTER_TYPED_TEST_CASE_P(VectorConverterTestCP, */
/*     ConvertsBetween */
/* ); */

/* /1* BLACK MAGIC, NO TOUCHY TOUCHY! //{ *1/ */

/* /1* template<typename ...Ts> struct type_list {}; *1/ */
/* template<typename ...Ts> */
/* using type_list = ::testing::Types<Ts...>; */
/* template<typename T1, typename T2> struct tpair */
/* { */
/*   using from_type = T1; */
/*   using to_type = T2; */
/* }; */

/* // Concatenation */
/* template <typename ... T> struct concat; */
/* template <typename ... Ts, typename ... Us> */
/* struct concat<type_list<Ts...>, type_list<Us...>> */
/* { */
/*     typedef type_list<Ts..., Us...> type; */
/* }; */

/* // Cross Product */
/* template <typename T, typename U> struct cross_product; */

/* // Partially specialise the empty case for the first type_list. */
/* template <typename ...Us> */
/* struct cross_product<type_list<>, type_list<Us...>> { */
/*     typedef type_list<> type; */
/* }; */

/* // The general case for two type_lists. Process: */
/* // 1. Expand out the head of the first type_list with the full second type_list. */
/* // 2. Recurse the tail of the first type_list. */
/* // 3. Concatenate the two type_lists. */
/* template <typename T, typename ...Ts, typename ...Us> */
/* struct cross_product<type_list<T, Ts...>, type_list<Us...>> { */
/*     typedef typename concat< */
/*         type_list<tpair<T, Us>...>, */
/*         typename cross_product<type_list<Ts...>, type_list<Us...>>::type */
/*     >::type type; */
/* }; */

/* //} */

/* /1* typedef ::testing::Types< *1/ */
/* /1* cross_product<type_list<VC_TYPE_LIST>, type_list<VC_TYPE_LIST>> *1/ */
/* /1*   > VectorCPTypes; *1/ */
/* using VectorCPTypes = cross_product<type_list<VC_TYPE_LIST>::type, type_list<VC_TYPE_LIST>::type>; */

/* INSTANTIATE_TYPED_TEST_CASE_P(VectorTypesInstantiation2, VectorConverterTestCP, VectorCPTypes); */

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

