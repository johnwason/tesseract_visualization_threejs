cmake_minimum_required(VERSION 3.16.0)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

# Plugins must be shared libraries, and the plugin links these
if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

option(TESSERACT_VIEWER_BUILD_SERVER "Build the Boost.Beast server and the Visualization plugin" ON)
option(TESSERACT_VIEWER_ENABLE_TESTING "Build the unit tests" OFF)

find_package(Eigen3 REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(tesseract REQUIRED COMPONENTS common geometry scene_graph state_solver collision kinematics environment
                                           visualization)
set(TESSERACT_VIEWER_TESSERACT_DEPS tesseract::common tesseract::geometry tesseract::scene_graph
                                    tesseract::environment tesseract::kinematics tesseract::visualization)

# The client and the primitive meshes are compiled in: wheels and other relocated installs then
# need no data files and no resource path setup.
set(RES ${CMAKE_CURRENT_SOURCE_DIR}/resources)
set(EMBEDDED ${CMAKE_CURRENT_BINARY_DIR}/embedded_resources_data.cpp)
add_custom_command(
  OUTPUT ${EMBEDDED}
  COMMAND ${CMAKE_COMMAND} -DOUTPUT=${EMBEDDED}
          "-DINPUTS=index_html=${RES}/static/index.html,app_js=${RES}/static/app.js,geometries_json=${RES}/geometries.json"
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_resources.cmake
  DEPENDS ${RES}/static/index.html ${RES}/static/app.js ${RES}/geometries.json
          ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_resources.cmake
  VERBATIM)

# Layers 1 and 2: conversion and transport independent viewer state
add_library(tesseract_viewer src/gltf.cpp src/trajectory.cpp src/markers.cpp src/viewer_state.cpp src/sha256.cpp
                             src/embedded_resources.cpp ${EMBEDDED})
target_compile_features(tesseract_viewer PUBLIC cxx_std_17)
target_compile_options(tesseract_viewer PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
target_link_libraries(tesseract_viewer PUBLIC Eigen3::Eigen ${TESSERACT_VIEWER_TESSERACT_DEPS}
                      PRIVATE JsonCpp::JsonCpp OpenSSL::Crypto)
target_include_directories(tesseract_viewer PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:include>")
set(TESSERACT_VIEWER_TARGETS tesseract_viewer)

if(TESSERACT_VIEWER_BUILD_SERVER)
  find_package(Boost REQUIRED)
  find_package(Threads REQUIRED)

  # Layer 3: HTTP and websocket server
  add_library(tesseract_viewer_server src/viewer_server.cpp)
  target_compile_options(tesseract_viewer_server PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
  target_link_libraries(tesseract_viewer_server PUBLIC tesseract_viewer PRIVATE Boost::boost Threads::Threads)

  # tesseract::visualization::Visualization plugin
  add_library(tesseract_viewer_visualization_plugin SHARED src/viewer_visualization_plugin.cpp)
  target_compile_options(tesseract_viewer_visualization_plugin PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
  target_link_libraries(tesseract_viewer_visualization_plugin PRIVATE tesseract_viewer_server)

  add_executable(tesseract_viewer_demo examples/viewer_demo.cpp)
  target_link_libraries(tesseract_viewer_demo PRIVATE tesseract_viewer_server)

  list(APPEND TESSERACT_VIEWER_TARGETS tesseract_viewer_server tesseract_viewer_visualization_plugin
       tesseract_viewer_demo)
endif()

configure_package(NAMESPACE tesseract TARGETS ${TESSERACT_VIEWER_TARGETS}
                  DEPENDENCIES Eigen3 jsoncpp OpenSSL
                  "tesseract COMPONENTS common geometry scene_graph state_solver collision kinematics environment visualization")
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")

if(TESSERACT_VIEWER_ENABLE_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(tesseract_viewer_unit test/viewer_unit.cpp)
  target_link_libraries(tesseract_viewer_unit PRIVATE tesseract_viewer JsonCpp::JsonCpp OpenSSL::Crypto GTest::gtest_main)
  add_test(NAME tesseract_viewer_unit COMMAND tesseract_viewer_unit)

  if(TESSERACT_VIEWER_BUILD_SERVER)
    add_executable(tesseract_viewer_server_unit test/server_unit.cpp)
    target_link_libraries(tesseract_viewer_server_unit PRIVATE tesseract_viewer_server JsonCpp::JsonCpp Boost::boost
                                                               Threads::Threads GTest::gtest_main)
    add_test(NAME tesseract_viewer_server_unit COMMAND tesseract_viewer_server_unit)

    # Driven by test/browser/viewer.spec.js, which is run separately by tools/test_browser.sh
    add_executable(tesseract_viewer_browser_fixture test/browser_fixture.cpp)
    target_link_libraries(tesseract_viewer_browser_fixture PRIVATE tesseract_viewer_server JsonCpp::JsonCpp)

    set_tests_properties(tesseract_viewer_server_unit PROPERTIES
      ENVIRONMENT "TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=$<TARGET_FILE_DIR:tesseract_viewer_visualization_plugin>;TESSERACT_VISUALIZATION_PLUGINS=tesseract_viewer_visualization_plugin")
  endif()
endif()
