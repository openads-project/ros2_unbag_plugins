// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <pybind11/pybind11.h>
#include <cloudini_lib/ros_msg_utils.hpp>

namespace py = pybind11;

/**
 * @brief Expose the upstream encoder for generating test fixtures in each wire format.
 * @param module Python module receiving the test-only encode function.
 */
PYBIND11_MODULE(_cloudini_test_encoder, module) {
  module.def("encode", [](const py::bytes& message, uint8_t version, bool lossy) {
    const std::string input = message;
    auto cloud = cloudini_ros::getDeserializedPointCloudMessage(
        {reinterpret_cast<const uint8_t*>(input.data()), input.size()});
    auto info = cloudini_ros::toEncodingInfo(cloud);
    info.version = version;
    info.encoding_opt = lossy ? Cloudini::EncodingOptions::LOSSY : Cloudini::EncodingOptions::LOSSLESS;
    if (lossy) {
      cloudini_ros::applyResolutionProfile({}, info.fields, 0.001f);
    }
    std::vector<uint8_t> output;
    cloudini_ros::convertPointCloud2ToCompressedCloud(cloud, info, output, nullptr, false);
    return py::bytes(reinterpret_cast<const char*>(output.data()), output.size());
  });
}
