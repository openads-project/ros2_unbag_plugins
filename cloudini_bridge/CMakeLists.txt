cmake_minimum_required(VERSION 3.20)
project(ros2_unbag_cloudini)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)

if(SKBUILD)
  # A pip wheel embeds the same upstream version as the Docker/colcon build.
  # No ROS workspace, system Cloudini installation, or runtime library path is needed.
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
      "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['repositories']['cloudini']['version'])"
      "${CMAKE_CURRENT_SOURCE_DIR}/../.repos"
    OUTPUT_VARIABLE CLOUDINI_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY)
  include(FetchContent)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(BUILD_SHARED_LIBS OFF)
  FetchContent_Declare(cloudini
    URL "https://github.com/facontidavide/cloudini/archive/refs/tags/${CLOUDINI_VERSION}.tar.gz"
    SOURCE_SUBDIR cloudini_lib)
  FetchContent_MakeAvailable(cloudini)
  # Retain the notices for the libraries embedded in the wheel.
  set(LICENSE_DIR "share/ros2_unbag_cloudini/licenses")
  install(FILES "${cloudini_SOURCE_DIR}/LICENSE" DESTINATION "${LICENSE_DIR}/cloudini")
  install(FILES "${CPM_PACKAGE_zstd_SOURCE_DIR}/LICENSE" DESTINATION "${LICENSE_DIR}/zstd")
  install(FILES "${CPM_PACKAGE_lz4_SOURCE_DIR}/lib/LICENSE" DESTINATION "${LICENSE_DIR}/lz4")
  set(CLOUDINI_TARGET cloudini::cloudini_lib)
else()
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(cloudini_lib 1.4.1 REQUIRED)
  set(CLOUDINI_TARGET cloudini_lib::cloudini_lib)
endif()

pybind11_add_module(_ros2_unbag_cloudini src/decoder.cpp)
target_compile_features(_ros2_unbag_cloudini PRIVATE cxx_std_17)
target_link_libraries(_ros2_unbag_cloudini PRIVATE ${CLOUDINI_TARGET})

if(SKBUILD)
  install(TARGETS _ros2_unbag_cloudini LIBRARY DESTINATION .)
else()
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  install(TARGETS _ros2_unbag_cloudini LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}")
  ament_environment_hooks(env/pythonpath.dsv.in)

  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    pybind11_add_module(_cloudini_test_encoder test/encoder.cpp)
    target_compile_features(_cloudini_test_encoder PRIVATE cxx_std_17)
    target_link_libraries(_cloudini_test_encoder PRIVATE ${CLOUDINI_TARGET})
    ament_add_pytest_test(cloudini_decoder test/test_decoder.py
      APPEND_ENV PYTHONPATH=$<TARGET_FILE_DIR:_ros2_unbag_cloudini>)
  endif()

  ament_package()
endif()
