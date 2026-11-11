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
  add_compile_definitions(NOMINMAX _WIN32_WINNT=0x0A00)
  add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/bigobj>)
endif()

option(TESSERACT_VISUALIZATION_THREEJS_BUILD_SERVER "Build the Boost.Beast server and the Visualization plugin" ON)
option(TESSERACT_VISUALIZATION_THREEJS_ENABLE_TESTING "Build the unit tests" OFF)

find_package(Eigen3 REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(tesseract REQUIRED COMPONENTS common geometry scene_graph state_solver collision kinematics environment
                                           visualization)
set(TESSERACT_VISUALIZATION_THREEJS_TESSERACT_DEPS tesseract::common tesseract::geometry tesseract::scene_graph
                                    tesseract::environment tesseract::kinematics tesseract::visualization)

# The client and the primitive meshes are compiled in: wheels and other relocated installs then
# need no data files and no resource path setup.
set(RES ${CMAKE_CURRENT_SOURCE_DIR}/resources)
set(EMBEDDED ${CMAKE_CURRENT_BINARY_DIR}/embedded_resources_data.cpp)
set(EMBEDDED_STATIC_FILES
    index.html
    app.js
    vendor/robust-websocket/robust-websocket.min.js
    vendor/three/build/three.module.js
    vendor/three/examples/jsm/controls/OrbitControls.js
    vendor/three/examples/jsm/environments/RoomEnvironment.js
    vendor/three/examples/jsm/libs/motion-controllers.module.js
    vendor/three/examples/jsm/lines/Line2.js
    vendor/three/examples/jsm/lines/LineGeometry.js
    vendor/three/examples/jsm/lines/LineMaterial.js
    vendor/three/examples/jsm/lines/LineSegments2.js
    vendor/three/examples/jsm/lines/LineSegmentsGeometry.js
    vendor/three/examples/jsm/loaders/GLTFLoader.js
    vendor/three/examples/jsm/utils/BufferGeometryUtils.js
    vendor/three/examples/jsm/webxr/VRButton.js
    vendor/three/examples/jsm/webxr/XRControllerModelFactory.js)
set(EMBEDDED_INPUTS "geometries.json=${RES}/geometries.json")
set(EMBEDDED_DEPENDS ${RES}/geometries.json)
foreach(file IN LISTS EMBEDDED_STATIC_FILES)
  string(APPEND EMBEDDED_INPUTS ",${file}=${RES}/static/${file}")
  list(APPEND EMBEDDED_DEPENDS ${RES}/static/${file})
endforeach()
add_custom_command(
  OUTPUT ${EMBEDDED}
  COMMAND ${CMAKE_COMMAND} -DOUTPUT=${EMBEDDED} "-DINPUTS=${EMBEDDED_INPUTS}"
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_resources.cmake
  DEPENDS ${EMBEDDED_DEPENDS} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_resources.cmake
  VERBATIM)

# Layers 1 and 2: conversion and transport independent viewer state
add_library(tesseract_visualization_threejs src/gltf.cpp src/trajectory.cpp src/markers.cpp src/viewer_state.cpp src/sha256.cpp
                             src/embedded_resources.cpp ${EMBEDDED})
target_compile_features(tesseract_visualization_threejs PUBLIC cxx_std_17)
target_compile_options(tesseract_visualization_threejs PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
target_link_libraries(tesseract_visualization_threejs PUBLIC Eigen3::Eigen ${TESSERACT_VISUALIZATION_THREEJS_TESSERACT_DEPS}
                      PRIVATE JsonCpp::JsonCpp OpenSSL::Crypto)
target_include_directories(tesseract_visualization_threejs PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories(tesseract_visualization_threejs PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:include>")
set(TESSERACT_VISUALIZATION_THREEJS_TARGETS tesseract_visualization_threejs)

if(TESSERACT_VISUALIZATION_THREEJS_BUILD_SERVER)
  find_package(Boost REQUIRED)
  find_package(Threads REQUIRED)

  # Layer 3: HTTP and websocket server
  add_library(tesseract_visualization_threejs_server src/viewer_server.cpp src/threejs_visualization.cpp)
  target_compile_options(tesseract_visualization_threejs_server PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
  target_link_libraries(tesseract_visualization_threejs_server PUBLIC tesseract_visualization_threejs PRIVATE Boost::headers Threads::Threads)

  # tesseract::visualization::Visualization plugin
  add_library(tesseract_visualization_threejs_plugin SHARED src/threejs_visualization_plugin.cpp)
  target_compile_options(tesseract_visualization_threejs_plugin PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
  target_link_libraries(tesseract_visualization_threejs_plugin PRIVATE tesseract_visualization_threejs_server)

  set(TESSERACT_VISUALIZATION_THREEJS_EXAMPLES)
  foreach(example IN ITEMS viewer_demo shapes_viewer material_mesh_viewer abb_irb2400_viewer)
    add_executable(tesseract_visualization_threejs_${example} examples/${example}.cpp)
    target_link_libraries(tesseract_visualization_threejs_${example} PRIVATE tesseract_visualization_threejs_server)
    list(APPEND TESSERACT_VISUALIZATION_THREEJS_EXAMPLES tesseract_visualization_threejs_${example})
  endforeach()

  list(APPEND TESSERACT_VISUALIZATION_THREEJS_TARGETS tesseract_visualization_threejs_server tesseract_visualization_threejs_plugin
       ${TESSERACT_VISUALIZATION_THREEJS_EXAMPLES})
endif()

configure_package(NAMESPACE tesseract TARGETS ${TESSERACT_VISUALIZATION_THREEJS_TARGETS}
                  DEPENDENCIES Eigen3 jsoncpp OpenSSL
                  "tesseract COMPONENTS common geometry scene_graph state_solver collision kinematics environment visualization")
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")

if(TESSERACT_VISUALIZATION_THREEJS_ENABLE_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  # tesseract installs its support data, which the tests load as package://tesseract/support
  get_filename_component(TESSERACT_SHARE_DIR "${tesseract_DIR}/../../../share" ABSOLUTE)
  add_executable(tesseract_visualization_threejs_unit test/viewer_unit.cpp)
  target_link_libraries(tesseract_visualization_threejs_unit PRIVATE tesseract_visualization_threejs JsonCpp::JsonCpp OpenSSL::Crypto GTest::gtest_main)
  add_test(NAME tesseract_visualization_threejs_unit COMMAND tesseract_visualization_threejs_unit)
  set_tests_properties(tesseract_visualization_threejs_unit PROPERTIES ENVIRONMENT "TESSERACT_RESOURCE_PATH=${TESSERACT_SHARE_DIR}")

  if(TESSERACT_VISUALIZATION_THREEJS_BUILD_SERVER)
    add_executable(tesseract_visualization_threejs_server_unit test/server_unit.cpp)
    target_link_libraries(tesseract_visualization_threejs_server_unit PRIVATE tesseract_visualization_threejs_server JsonCpp::JsonCpp Boost::headers
                                                               Threads::Threads GTest::gtest_main)
    add_test(NAME tesseract_visualization_threejs_server_unit COMMAND tesseract_visualization_threejs_server_unit)

    # Driven by test/browser/viewer.spec.js, which is run separately by tools/test_browser.sh
    add_executable(tesseract_visualization_threejs_browser_fixture test/browser_fixture.cpp)
    target_link_libraries(tesseract_visualization_threejs_browser_fixture PRIVATE tesseract_visualization_threejs_server JsonCpp::JsonCpp)

    set_tests_properties(tesseract_visualization_threejs_server_unit PROPERTIES
      ENVIRONMENT "TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=$<TARGET_FILE_DIR:tesseract_visualization_threejs_plugin>;TESSERACT_VISUALIZATION_PLUGINS=tesseract_visualization_threejs_plugin")
  endif()
endif()
