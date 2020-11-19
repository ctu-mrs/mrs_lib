#include <mrs_lib/vector_converter.h>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

using namespace std;

template <typename T>
class VectorConverterTest: public ::testing::Test { };
TYPED_TEST_CASE_P(VectorConverterTest);

TYPED_TEST_P(VectorConverterTest, ConvertsTo)
{
  const TypeParam vec = mrs_lib::convertTo<TypeParam>(rand(), rand(), rand());;
}

TYPED_TEST_P(VectorConverterTest, ConvertsFrom)
{
  const TypeParam vec;
  const auto [x, y, z] = mrs_lib::convertFrom(vec);
}

TYPED_TEST_P(VectorConverterTest, ConvertsValid)
{
  const double xo = rand(), yo = rand(), zo = rand();
  const TypeParam vec = mrs_lib::convertTo<TypeParam>(xo, yo, zo);;
  const auto [x, y, z] = mrs_lib::convertFrom(vec);
  EXPECT_TRUE(x == xo && y == yo && z == zo);
}

REGISTER_TYPED_TEST_CASE_P(VectorConverterTest,
    ConvertsTo,
    ConvertsFrom,
    ConvertsValid
);

typedef ::testing::Types<cv::Vec3d> VectorTypes;
INSTANTIATE_TYPED_TEST_CASE_P(VectorTypesInstantiation, VectorConverterTest, VectorTypes);

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

