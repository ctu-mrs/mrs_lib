// this file is just for documentation purposes and contains no actual code

#include <mrs_lib/internal/version_macros.hpp>

/**
 * \brief All mrs_lib functions, classes, variables and definitions are contained in this namespace.
 *
 */
namespace mrs_lib
{

  /**
   * @brief Namespace for api v1. Definitions from this namespace are available
   * in `mrs_lib::` directly when using `MRS_LIB_API_VERSION == 1`.
   */
  MRS_LIB_INTERNAL_INLINE_API_V1 namespace v1
  {

    /**
     * @brief This namespace contains implementation details that should not be
     * used by library consumers.
     */
    namespace internal
    {

    } // namespace internal

  } // namespace v1

  /**
   * @brief Namespace for api v2. Definitions from this namespace are available
   * in `mrs_lib::` directly when using `MRS_LIB_API_VERSION == 2`.
   */
  MRS_LIB_INTERNAL_INLINE_API_V2 namespace v2
  {

    /**
     * @brief This namespace contains implementation details that should not be
     * used by library consumers.
     */
    namespace internal
    {

    } // namespace internal

  } // namespace v2


} // namespace mrs_lib
