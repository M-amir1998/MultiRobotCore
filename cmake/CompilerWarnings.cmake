# Project warning policy. Applied PRIVATE to project targets only, so that
# downstream consumers never inherit these flags.

include_guard(GLOBAL)

function(mrc_set_project_warnings target)
  set(_gnu_warnings
      -Wall
      -Wextra
      -Wpedantic
      -Wshadow
      -Wnon-virtual-dtor
      -Wold-style-cast
      -Wcast-align
      -Woverloaded-virtual
      -Wnull-dereference
      -Wdouble-promotion
      -Wformat=2
      -Wimplicit-fallthrough)

  set(_msvc_warnings
      /W4
      /permissive-
      /w14242 # conversion, possible loss of data
      /w14254 # larger bit field assigned to smaller
      /w14263 # member function does not override base virtual
      /w14265 # class has virtual functions but non-virtual destructor
      /w14287 # unsigned/negative constant mismatch
      /we4289 # loop control variable used outside the loop
      /w14296 # expression is always true/false
      /w14311 # pointer truncation
      /w14545 /w14546 /w14547 /w14549 # suspicious comma/operator expressions
      /w14555 # expression has no effect
      /w14640 # thread-unsafe static member initialization
      /w14826 # sign-extended conversion
      /w14905 /w14906 # string literal casts
      /w14928) # illegal copy-initialization

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
    # clang-cl: /W4 maps to -Wall -Wextra. Note that clang-cl's -Wall would
    # enable -Weverything, so GNU-style extras are added explicitly instead.
    set(_warnings
        /W4
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Woverloaded-virtual
        -Wnull-dereference
        -Wdouble-promotion
        -Wimplicit-fallthrough
        -Wno-c++98-compat
        -Wno-c++98-compat-pedantic)
    if(MULTIROBOTCORE_WARNINGS_AS_ERRORS)
      list(APPEND _warnings /WX)
    endif()
  elseif(MSVC)
    set(_warnings ${_msvc_warnings})
    if(MULTIROBOTCORE_WARNINGS_AS_ERRORS)
      list(APPEND _warnings /WX)
    endif()
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    set(_warnings ${_gnu_warnings})
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      list(APPEND _warnings -Wmisleading-indentation -Wduplicated-cond -Wlogical-op)
      # GCC's -Wnull-dereference reports false positives inside libstdc++
      # containers at -O2 (GCC 12/13); it stays enabled for Clang.
      list(REMOVE_ITEM _warnings -Wnull-dereference)
    endif()
    if(MULTIROBOTCORE_WARNINGS_AS_ERRORS)
      list(APPEND _warnings -Werror)
    endif()
  endif()

  target_compile_options(${target} PRIVATE ${_warnings})
  if(MSVC)
    target_compile_options(${target} PRIVATE /utf-8 /Zc:__cplusplus /bigobj)
    target_compile_definitions(${target} PRIVATE _USE_MATH_DEFINES NOMINMAX)
  endif()
endfunction()
