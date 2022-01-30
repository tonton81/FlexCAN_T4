cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(FlexCan
    VERSION 1.0.0
    DESCRIPTION "FlexCAN driver"
    LANGUAGES CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY 	https://github.com/bolderflight/core.git
    GIT_TAG v3.0.2
  )
  FetchContent_MakeAvailable(core)
  # Add the library target
  add_library(flexcan
    src/circular_buffer.h
    src/FlexCAN_T4.h
    src/FlexCAN_T4.tpp
    src/FlexCAN_T4FD.tpp
    src/FlexCAN_T4FDTimings.tpp
    src/imxrt_flexcan.h
    src/kinetis_flexcan.h
    src/isotp_server.h
    src/isotp_server.tpp
    src/isotp.h
    src/isotp.tpp
    src/random.h
    src/random.cpp
    src/constrain.h
  )
  # Link libraries
  target_link_libraries(flexcan
    PUBLIC
      core
  )
  # Setup include directories 
  target_include_directories(flexcan PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # Add the example target
    add_executable(loopback_example examples/cmake/loopback.cc)
    # Add the includes
    target_include_directories(loopback_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(loopback_example
      PRIVATE 
        flexcan
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(loopback_example ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
