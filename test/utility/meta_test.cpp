#include "mrs_lib/utility/meta.hpp"

#include <gtest/gtest.h>

#include <concepts>
#include <iostream>
#include <tuple>
#include <variant>


namespace
{

  using EmptyTypeList = mrs_lib::meta::TypeList<>;
  using TypeList1 = mrs_lib::meta::TypeList<int, double, std::string, char, void*>;

  //////////////////////////////////////////////////////////////////////////////
  //  Drop                                                                    //
  //////////////////////////////////////////////////////////////////////////////

  static_assert(std::same_as<mrs_lib::meta::DropT<TypeList1, 0>, mrs_lib::meta::TypeList<int, double, std::string, char, void*>>);
  static_assert(std::same_as<mrs_lib::meta::DropT<TypeList1, 1>, mrs_lib::meta::TypeList<double, std::string, char, void*>>);
  static_assert(std::same_as<mrs_lib::meta::DropT<TypeList1, 2>, mrs_lib::meta::TypeList<std::string, char, void*>>);
  static_assert(std::same_as<mrs_lib::meta::DropT<TypeList1, 3>, mrs_lib::meta::TypeList<char, void*>>);
  static_assert(std::same_as<mrs_lib::meta::DropT<TypeList1, 4>, mrs_lib::meta::TypeList<void*>>);
  static_assert(std::same_as<mrs_lib::meta::DropT<TypeList1, 5>, mrs_lib::meta::TypeList<>>);

  static_assert(std::same_as<mrs_lib::meta::DropT<EmptyTypeList, 0>, EmptyTypeList>);

  //////////////////////////////////////////////////////////////////////////////
  //  Apply                                                                   //
  //////////////////////////////////////////////////////////////////////////////

  static_assert(std::same_as<mrs_lib::meta::ApplyT<std::tuple, EmptyTypeList>, std::tuple<>>);
  static_assert(std::same_as<mrs_lib::meta::ApplyT<std::tuple, TypeList1>, std::tuple<int, double, std::string, char, void*>>);

  static_assert(std::same_as<mrs_lib::meta::ApplyT<std::variant, EmptyTypeList>, std::variant<>>);
  static_assert(std::same_as<mrs_lib::meta::ApplyT<std::variant, TypeList1>, std::variant<int, double, std::string, char, void*>>);


  // Just to have something in the log :)
  TEST(UtilityMeta, Test)
  {
    std::cout << "mrs_lib::meta tests are done in compile time.\n";
    EXPECT_EQ(42, 42);
  }

} // namespace
