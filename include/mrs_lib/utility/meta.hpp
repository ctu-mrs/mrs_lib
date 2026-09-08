#ifndef MRS_LIB_UTILITY_META_HPP_
#define MRS_LIB_UTILITY_META_HPP_

#include <cstddef>


/**
 * @brief Namespace containing metaprograming utilities.
 */
namespace mrs_lib::meta
{

  /**
   * @brief Struct for storing compile time list of types.
   */
  template <typename...>
  struct TypeList
  {
  };

  /**
   * @brief Metafunction to drop first N types from TypeList.
   *
   * @tparam List TypeList from which to drop the types
   * @tparam N How many types to drop.
   *
   * @see DropT
   */
  template <typename List, size_t N>
  struct Drop; // Undefined

  template <typename... Ts>
  struct Drop<TypeList<Ts...>, size_t(0)>
  {
    using type = TypeList<Ts...>;
  };

  template <typename T, typename... Ts, size_t N>
    requires(N > 0)
  struct Drop<TypeList<T, Ts...>, N>
  {
    using type = Drop<TypeList<Ts...>, N - 1>::type;
  };

  /**
   * @brief Helper type alias for Drop<List, N>::type.
   */
  template <typename List, size_t N>
  using DropT = Drop<List, N>::type;

  /**
   * @brief Apply metafunction to TypeList of argument types.
   *
   * @tparam F Metafunction to apply to the types.
   * @tparam List Types passed as arguments to the specified metafunction.
   *
   * For `TypeList<Ts...>` it has member typedef `type` equal to `F<Ts...>`.
   */
  template <template <typename...> typename F, typename List>
  struct Apply; // Undefined

  template <template <typename...> typename F, typename... Args>
  struct Apply<F, TypeList<Args...>>
  {
    using type = F<Args...>;
  };

  /**
   * @brief Helper type alias for Apply<F, List>::type.
   */
  template <template <typename...> typename F, typename List>
  using ApplyT = Apply<F, List>::type;

} // namespace mrs_lib::meta


#endif // MRS_LIB_UTILITY_META_HPP_
