#include "mrs_lib/coro/runners.hpp"

#include <exception>
#include <iostream>
#include <format>
#include <ostream>
#include <string>


namespace mrs_lib::coro
{

  namespace internal
  {

    void AsyncRun::promise_type::unhandled_exception()
    {
      std::string msg;

      try
      {
        // Rethrow the unhandled exception.
        throw;
      }
      catch (const std::exception& e)
      {
        msg = std::format("ERROR: Unhandled exception in AsyncRun coroutine.\n  e.what(): {}", e.what());
      }
      catch (...)
      {
        msg = std::format("ERROR: Unhandled exception in AsyncRun coroutine.");
      }

      std::cerr << msg << "\n" << std::flush;
      std::terminate();
    }


  } // namespace internal


} // namespace mrs_lib::coro
