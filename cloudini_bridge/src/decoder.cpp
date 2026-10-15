// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <pybind11/pybind11.h>
#include <cloudini_lib/ros_msg_utils.hpp>

namespace py = pybind11;

/**
 * @brief Expose upstream Cloudini decoding of serialized ROS point clouds to Python.
 * @param module Python module receiving the decode function.
 */
PYBIND11_MODULE(_ros2_unbag_cloudini, module) {
  module.doc() = "Decode Cloudini using the installed upstream library.";
  module.def("decode", [](const py::bytes& message) {
    const std::string input = message;
    std::vector<uint8_t> output;
    {
      py::gil_scoped_release release;
      const auto cloud = cloudini_ros::getDeserializedPointCloudMessage(
          {reinterpret_cast<const uint8_t*>(input.data()), input.size()});
      cloudini_ros::convertCompressedCloudToPointCloud2(cloud, output);
    }
    return py::bytes(reinterpret_cast<const char*>(output.data()), output.size());
  }, py::arg("message"));
}
