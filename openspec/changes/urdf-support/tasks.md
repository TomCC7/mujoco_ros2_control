## 1. Model Source Resolution

- [x] 1.1 Add a helper to read optional string parameters from the existing `mujoco_ros2_control` node without throwing when a parameter is absent.
- [x] 1.2 Add a model-source resolver that preserves non-empty `mujoco_model_path` as the highest-priority source.
- [x] 1.3 Add fallback resolution for non-empty `robot_description` from the same running node context when no model path is provided.
- [x] 1.4 Define startup error messages for the case where neither `mujoco_model_path` nor non-empty `robot_description` is available.

## 2. URDF Preparation for MuJoCo

- [x] 2.1 Add the required `ament_index_cpp` build and package dependencies for resolving ROS package share directories.
- [x] 2.2 Implement URDF preprocessing that resolves `package://` mesh filenames to MuJoCo-resolvable filesystem paths.
- [x] 2.3 Add clear failure handling for unresolved `package://` URIs before MuJoCo compilation begins.
- [x] 2.4 Keep preprocessing scoped to startup model loading and avoid any work in controller read/update/write paths.

## 3. MuJoCo Model Loading

- [x] 3.1 Refactor existing model loading in `mujoco_ros2_control_node.cpp` into a single path that returns a valid `mjModel*` before `mjData` creation.
- [x] 3.2 Preserve `.mjb` loading through `mj_loadModel` and XML/MJCF file loading through `mj_loadXML`.
- [x] 3.3 Implement URDF string parsing and compilation using MuJoCo's in-memory XML/spec APIs when available.
- [x] 3.4 Add a compatible fallback path if the supported MuJoCo version does not expose the planned in-memory XML/spec APIs.
- [x] 3.5 Log MuJoCo parse and compile errors with enough detail to identify invalid URDF or unsupported URDF constructs.

## 4. Shared URDF Control Initialization

- [x] 4.1 Update `MujocoRos2Control` construction or initialization to accept an optional already-resolved URDF string.
- [x] 4.2 Use the provided URDF string for `parse_control_resources_from_urdf`, ResourceManager component import, and `urdf::Model::initString` when URDF loading is selected.
- [x] 4.3 Preserve the existing parameter/topic `robot_description` lookup as a backward-compatible fallback when no resolved URDF is provided.
- [x] 4.4 Ensure MuJoCo model compilation and ros2_control initialization cannot use different URDF contents in the URDF-loading path.

## 5. Examples and Documentation

- [x] 5.1 Add or update a launch example that starts `mujoco_ros2_control` with `robot_description` and without `mujoco_model_path`.
- [x] 5.2 Update documentation to describe accepted model sources, precedence, and the node-scoped nature of `robot_description`.
- [x] 5.3 Document URDF limitations and recommend MJCF/MJB for users who need advanced MuJoCo-specific modeling features.

## 6. Validation

- [x] 6.1 Build the affected package successfully after dependency and source changes.
- [x] 6.2 Run or smoke-test an existing XML/MJCF launch path to verify backward compatibility.
- [x] 6.3 Run or smoke-test an existing `.mjb` path if a demo or fixture is available.
- [x] 6.4 Run or smoke-test the new URDF-only `robot_description` launch path.
- [x] 6.5 Verify failure behavior for missing model sources, invalid URDF content, and unresolved `package://` mesh URIs.
