// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <array>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <limits.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;

namespace
{
struct LoadedModel
{
  mjModel *model = nullptr;
  std::string robot_description;
};

bool has_suffix(const std::string &value, const std::string &suffix)
{
  if (value.size() < suffix.size())
  {
    return false;
  }
  return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool is_safe_package_relative_path(const std::string &relative_path)
{
  if (relative_path.empty() || relative_path[0] == '/')
  {
    return false;
  }

  std::size_t cursor = 0;
  while (cursor <= relative_path.size())
  {
    const auto separator = relative_path.find('/', cursor);
    const std::string component = relative_path.substr(
      cursor, separator == std::string::npos ? std::string::npos : separator - cursor);
    if (component.empty() || component == "." || component == "..")
    {
      return false;
    }
    if (separator == std::string::npos)
    {
      break;
    }
    cursor = separator + 1;
  }

  return true;
}

std::string canonical_path(const std::string &path, const std::string &description)
{
  char resolved_path[PATH_MAX];
  if (realpath(path.c_str(), resolved_path) == nullptr)
  {
    throw std::runtime_error("Unable to resolve " + description + ": " + path);
  }
  return std::string(resolved_path);
}

bool is_path_inside_directory(const std::string &path, const std::string &directory)
{
  return path.size() > directory.size() && path.compare(0, directory.size(), directory) == 0 &&
         path[directory.size()] == '/';
}

bool get_optional_string_parameter(
  const rclcpp::Node::SharedPtr &node, const std::string &name, std::string &value)
{
  if (!node->has_parameter(name))
  {
    return false;
  }

  try
  {
    value = node->get_parameter(name).as_string();
  }
  catch (const rclcpp::ParameterTypeException &ex)
  {
    throw std::runtime_error("Parameter '" + name + "' must be a string: " + ex.what());
  }
  return true;
}

std::string resolve_package_uri(const std::string &uri)
{
  const std::string prefix = "package://";
  const std::string package_path = uri.substr(prefix.size());
  const auto separator = package_path.find('/');
  if (separator == std::string::npos || separator == 0 || separator + 1 >= package_path.size())
  {
    throw std::runtime_error("Invalid package URI: " + uri);
  }

  const std::string package_name = package_path.substr(0, separator);
  const std::string relative_path = package_path.substr(separator + 1);
  if (!is_safe_package_relative_path(relative_path))
  {
    throw std::runtime_error("Unsafe package URI path: " + uri);
  }

  const std::string package_share_directory =
    ament_index_cpp::get_package_share_directory(package_name);
  const std::string package_share_path =
    canonical_path(package_share_directory, "package share directory for '" + package_name + "'");
  const std::string resolved_path = canonical_path(
    package_share_directory + "/" + relative_path, "package URI target '" + uri + "'");
  if (!is_path_inside_directory(resolved_path, package_share_path))
  {
    throw std::runtime_error("Package URI resolves outside package share directory: " + uri);
  }

  return resolved_path;
}

std::string resolve_package_uris(const std::string &xml)
{
  const std::string prefix = "package://";
  std::string resolved;
  resolved.reserve(xml.size());

  std::size_t cursor = 0;
  while (cursor < xml.size())
  {
    const auto package_pos = xml.find(prefix, cursor);
    if (package_pos == std::string::npos)
    {
      resolved.append(xml, cursor, std::string::npos);
      break;
    }

    resolved.append(xml, cursor, package_pos - cursor);
    const auto uri_end = xml.find_first_of("\"'<> \t\r\n", package_pos);
    const std::string uri = xml.substr(
      package_pos, uri_end == std::string::npos ? std::string::npos : uri_end - package_pos);
    resolved += resolve_package_uri(uri);

    if (uri_end == std::string::npos)
    {
      break;
    }
    cursor = uri_end;
  }

  return resolved;
}

mjModel *load_model_from_file(const std::string &model_path, rclcpp::Logger logger)
{
  std::array<char, 1000> error{};
  error[0] = '\0';

  if (has_suffix(model_path, ".mjb"))
  {
    return mj_loadModel(model_path.c_str(), nullptr);
  }

  mjModel *model = mj_loadXML(model_path.c_str(), nullptr, error.data(), error.size());
  if (!model && error[0] != '\0')
  {
    RCLCPP_ERROR(logger, "Failed to load MuJoCo XML/URDF file '%s': %s", model_path.c_str(), error.data());
  }
  return model;
}

#if !defined(mjVERSION_HEADER) || mjVERSION_HEADER < 320
mjModel *load_model_from_temporary_urdf(const std::string &robot_description, rclcpp::Logger logger)
{
  std::array<char, 1000> error{};
  char filename[] = "/tmp/mujoco_ros2_control_urdf_XXXXXX";
  const int fd = mkstemp(filename);
  if (fd == -1)
  {
    throw std::runtime_error("Failed to create temporary URDF file for MuJoCo loading");
  }

  {
    std::ofstream output(filename);
    if (!output)
    {
      close(fd);
      std::remove(filename);
      throw std::runtime_error("Failed to open temporary URDF file for writing");
    }
    output << robot_description;
  }
  close(fd);

  mjModel *model = mj_loadXML(filename, nullptr, error.data(), error.size());
  std::remove(filename);
  if (!model)
  {
    RCLCPP_ERROR(logger, "Failed to compile robot_description URDF with MuJoCo: %s", error.data());
  }
  return model;
}
#endif

mjModel *load_model_from_urdf_string(const std::string &robot_description, rclcpp::Logger logger)
{
#if defined(mjVERSION_HEADER) && mjVERSION_HEADER >= 320
  std::array<char, 1000> error{};
  mjSpec *spec = mj_parseXMLString(robot_description.c_str(), nullptr, error.data(), error.size());
  if (!spec)
  {
    RCLCPP_ERROR(logger, "Failed to parse robot_description URDF with MuJoCo: %s", error.data());
    return nullptr;
  }

  mjModel *model = mj_compile(spec, nullptr);
  if (!model)
  {
    RCLCPP_ERROR(logger, "Failed to compile robot_description URDF with MuJoCo: %s", mjs_getError(spec));
  }
  mj_deleteSpec(spec);
  return model;
#else
  return load_model_from_temporary_urdf(robot_description, logger);
#endif
}

LoadedModel load_mujoco_model(const rclcpp::Node::SharedPtr &node)
{
  LoadedModel loaded_model;
  const auto logger = node->get_logger();

  std::string model_path;
  try
  {
    if (get_optional_string_parameter(node, "mujoco_model_path", model_path) && !model_path.empty())
    {
      loaded_model.model = load_model_from_file(model_path, logger);
      return loaded_model;
    }
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(logger, "%s", ex.what());
    return loaded_model;
  }

  std::string robot_description;
  try
  {
    if (!get_optional_string_parameter(node, "robot_description", robot_description) || robot_description.empty())
    {
      RCLCPP_ERROR(
        logger,
        "No MuJoCo model source provided. Set non-empty 'mujoco_model_path' or "
        "non-empty 'robot_description' on the mujoco_ros2_control node.");
      return loaded_model;
    }
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(logger, "%s", ex.what());
    return loaded_model;
  }

  try
  {
    loaded_model.robot_description = resolve_package_uris(robot_description);
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(logger, "Failed to resolve robot_description resources for MuJoCo: %s", ex.what());
    return loaded_model;
  }

  loaded_model.model = load_model_from_urdf_string(loaded_model.robot_description, logger);
  return loaded_model;
}
}  // namespace

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");

  const LoadedModel loaded_model = load_mujoco_model(node);
  mujoco_model = loaded_model.model;
  if (!mujoco_model)
  {
    mju_error("Load model error");
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  // initialize mujoco control
  auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(
    node, mujoco_model, mujoco_data, loaded_model.robot_description);

  mujoco_control.init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco visualization environment for rendering and cameras
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }
  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  rendering->init(mujoco_model, mujoco_data);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

  auto cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
  cameras->init(mujoco_model);

  // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
  mjtNum last_cam_update = mujoco_data->time;
  while (rclcpp::ok() && !rendering->is_close_flag_raised())
  {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = mujoco_data->time;
    while (mujoco_data->time - simstart < 1.0 / 60.0)
    {
      mujoco_control.update();
    }
    rendering->update();

    // Updating cameras at ~6 Hz
    // TODO(eholum): Break control and rendering into separate processes
    if (simstart - last_cam_update > 1.0 / 6.0)
    {
      cameras->update(mujoco_model, mujoco_data);
      last_cam_update = simstart;
    }
  }

  rendering->close();
  cameras->close();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 1;
}
