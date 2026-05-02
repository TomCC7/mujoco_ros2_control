## Why

`mujoco_ros2_control` currently requires a MuJoCo XML or MJB model path even though the ROS 2 control stack is already driven by `robot_description` URDF content. Supporting URDF as a first-class model source will reduce duplicate robot descriptions and address the documented future-work goal of loading models directly from URDF.

## What Changes

- Add support for initializing the simulation from URDF content instead of requiring a pre-converted MuJoCo XML file.
- Resolve URDF content from existing hardware or node parameters when available, and fall back to reading the standard `robot_description` ROS parameter directly when the local value is empty.
- Keep the existing MuJoCo XML/MJB path flow available so current launch files continue to work.
- Report clear errors when no usable model description is available or when URDF-to-MuJoCo model loading fails.

## Capabilities

### New Capabilities

- `urdf-model-loading`: Defines how the node accepts URDF/`robot_description` input, resolves it from ROS parameters, and uses it as a supported source for MuJoCo simulation initialization.

### Modified Capabilities

- None.

## Impact

- Affects model initialization in `mujoco_ros2_control/src/mujoco_ros2_control_node.cpp` and robot description handling in `mujoco_ros2_control/src/mujoco_ros2_control.cpp`.
- May require updates to launch/config examples so users can provide `robot_description` directly instead of only `mujoco_model_path`.
- Depends on ROS 2 parameter behavior for `robot_description` and MuJoCo APIs capable of loading/compiling model content from URDF-derived input.
- Existing XML/MJB model loading should remain backward compatible.
