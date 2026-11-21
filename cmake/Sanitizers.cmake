# Global sanitizer instrumentation.
#
# Sanitizers must instrument every translation unit that shares objects with
# project code (including GoogleTest), so the flags are applied directory-wide
# before any target is created.

include_guard(GLOBAL)

if(NOT MULTIROBOTCORE_SANITIZERS)
  return()
endif()

set(_mrc_supported address undefined)
foreach(_san IN LISTS MULTIROBOTCORE_SANITIZERS)
  if(NOT _san IN_LIST _mrc_supported)
    message(FATAL_ERROR "Unsupported sanitizer '${_san}'. Supported: ${_mrc_supported}")
  endif()
endforeach()
list(JOIN MULTIROBOTCORE_SANITIZERS "," _mrc_san_list)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
  # CMake links clang-cl targets with lld-link directly, so the sanitizer
  # runtime is linked explicitly. The dynamic ASan runtime also exports the
  # UBSan handlers. The clang ASan runtime does not support the debug CRT, so
  # the release CRT is forced; the runtime DLL is copied next to executables
  # by mrc_copy_sanitizer_runtime().
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
  add_compile_options(-fsanitize=${_mrc_san_list} -fno-sanitize-recover=undefined /Oy-)
  execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" /clang:-print-resource-dir
    OUTPUT_VARIABLE _mrc_clang_resource_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(MULTIROBOTCORE_SANITIZER_RUNTIME_DIR "${_mrc_clang_resource_dir}/lib/windows")
  add_link_options("/LIBPATH:${MULTIROBOTCORE_SANITIZER_RUNTIME_DIR}"
                   clang_rt.asan_dynamic-x86_64.lib
                   /wholearchive:clang_rt.asan_dynamic_runtime_thunk-x86_64.lib)
elseif(MSVC)
  if(NOT MULTIROBOTCORE_SANITIZERS STREQUAL "address")
    message(FATAL_ERROR "MSVC supports only the 'address' sanitizer. Use clang-cl for UBSan.")
  endif()
  add_compile_options(/fsanitize=address /Zi)
  add_link_options(/INCREMENTAL:NO)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(-fsanitize=${_mrc_san_list} -fno-sanitize-recover=undefined
                      -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${_mrc_san_list})
else()
  message(FATAL_ERROR "Sanitizers are not supported for ${CMAKE_CXX_COMPILER_ID}")
endif()

message(STATUS "MultiRobotCore: sanitizers enabled: ${_mrc_san_list}")
