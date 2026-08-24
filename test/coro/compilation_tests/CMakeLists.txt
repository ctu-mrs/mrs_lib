function(mrs_lib_add_compilation_test label target file)
  if(TARGET "${target}")
    message(FATAL_ERROR "Target: '${target}' already exists.")
  endif()

  add_library("${target}" STATIC)
  set_target_properties("${target}"
    PROPERTIES
      EXCLUDE_FROM_ALL ON
      EXCLUDE_FROM_DEFAULT_BUILD ON
  )
  target_sources("${target}"
    PRIVATE
      "${file}"
  )
  target_link_libraries("${target}"
    PRIVATE
      mrs_lib::mrs_lib
  )

  ament_add_test("${target}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND
      $<TARGET_FILE:Python3::Interpreter>
      "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/runner.py"
      --cmake-command "${CMAKE_COMMAND}"
      --target "${target}"
      --source "${CMAKE_CURRENT_LIST_DIR}/${file}"
    TIMEOUT "${timeout}"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  )

  set_property(TEST "${target}"
    APPEND PROPERTY LABELS "${label}"
  )
endfunction()

mrs_lib_add_compilation_test("coro;coro_compile" test_compile_task_await_after_manual_op_lvalue "task_await_after_manual_op_lvalue.cpp")
mrs_lib_add_compilation_test("coro;coro_compile" test_compile_task_await_after_manual_op_xvalue "task_await_after_manual_op_xvalue.cpp")

mrs_lib_add_compilation_test("coro;coro_compile" test_compile_task_await_lvalue "task_await_lvalue.cpp")
mrs_lib_add_compilation_test("coro;coro_compile" test_compile_task_await_prvalue "task_await_prvalue.cpp")
mrs_lib_add_compilation_test("coro;coro_compile" test_compile_task_await_xvalue "task_await_xvalue.cpp")
