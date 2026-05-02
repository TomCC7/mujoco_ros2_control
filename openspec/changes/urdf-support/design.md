## Context

The executable currently loads the physics model in `mujoco_ros2_control/src/mujoco_ros2_control_node.cpp` from `mujoco_model_path`, using `mj_loadModel` for `.mjb` and `mj_loadXML` for everything else. Separately, `mujoco_ros2_control/src/mujoco_ros2_control.cpp` reads `robot_description`, parses ros2_control resources from that URDF, loads a `hardware_interface::ResourceManager`, and passes a parsed `urdf::Model` into `MujocoSystem::init_sim`.

The demo launch files already pass both `robot_description` and `mujoco_model_path` to the same `mujoco_ros2_control` node, so the package has two model-description sources that users must keep in sync. MuJoCo supports loading XML models in MJCF or URDF format with `mj_loadXML`; the format is inferred from the root element (`<mujoco>` for MJCF, `<robot>` for URDF), not from the filename extension. MuJoCo also exposes `mj_parseXMLString` and `mj_compile`, which allow compiling URDF content already available as a string.

ROS 2 parameters are node-scoped, not global. A temporary node named only for loading parameters will not reliably read another node's `robot_description` unless that temporary node also receives the parameter override. The design should therefore resolve `robot_description` through the already-running `mujoco_ros2_control_node` and reuse the same resolved string for both MuJoCo model creation and ros2_control initialization.

## Goals / Non-Goals

**Goals:**

- Keep existing `.xml` and `.mjb` model-path launches working without behavior changes.
- Allow users to omit `mujoco_model_path` and initialize MuJoCo from the node's `robot_description` parameter.
- Use one resolved URDF string for both MuJoCo model compilation and ros2_control hardware parsing/component import.
- Provide clear errors for missing parameters, empty descriptions, unsupported MuJoCo APIs, package URI resolution failures, and MuJoCo parse/compile errors.
- Keep model loading outside the real-time control loop.

**Non-Goals:**

- Do not implement a generic URDF-to-MJCF conversion pipeline outside MuJoCo's native URDF parser.
- Do not remove `mujoco_model_path` or require existing demos to migrate immediately.
- Do not attempt to read arbitrary parameters from unrelated ROS 2 nodes as if there were a global parameter server.
- Do not add runtime model reloading or live parameter updates in this change.
- Do not guarantee full MJCF feature parity for URDF input; MuJoCo's URDF support is more limited than MJCF.

## Decisions

### 1. Centralize robot description resolution on the main node

Add a small helper in the node/control layer that resolves model-description input in this order:

1. If `mujoco_model_path` is set and non-empty, use the existing path-based loading flow.
2. Otherwise, read `robot_description` from the existing `mujoco_ros2_control_node` parameter set.
3. If the parameter exists but is empty, optionally wait for the existing transient-local `robot_description` topic fallback used by `MujocoRos2Control::get_robot_description()`.
4. If no source produces content, fail initialization with a message that names both accepted inputs.

Rationale: launch files already pass `robot_description` to the main node, and using the same node avoids misleading temporary-node behavior. It also prevents MuJoCo from compiling one URDF while ros2_control parses a different URDF.

Alternatives considered:

- **Temporary loader node:** rejected because ROS 2 parameters are node-scoped and the temporary node will not automatically see another node's parameters.
- **Always subscribe to `robot_description`:** useful as a fallback, but slower and less deterministic than directly using the node parameter already supplied by launch.
- **Require `mujoco_model_path` forever:** rejected because it preserves the duplicate-description problem this change is meant to solve.

### 2. Introduce a model-source abstraction before creating `mjData`

Refactor the top of `mujoco_ros2_control_node.cpp` so model loading is handled by one function that returns an `mjModel*` plus, when available, the resolved URDF string:

- Path source:
  - `.mjb` -> `mj_loadModel(path.c_str(), nullptr)`.
  - other files -> `mj_loadXML(path.c_str(), nullptr, error, error_size)`.
- URDF string source:
  - normalize the URDF string for MuJoCo asset loading.
  - parse with `mj_parseXMLString(urdf.c_str(), nullptr, error, error_size)`.
  - compile with `mj_compile(spec, nullptr)`.
  - surface `mjs_getError(spec)` when compilation fails.
  - release the `mjSpec` after compilation.

Only create `mjData` after a valid `mjModel*` is returned.

Rationale: `mj_loadXML` can load URDF files from disk, but `robot_description` is an in-memory string. `mj_parseXMLString` plus `mj_compile` avoids temporary files and uses MuJoCo's native parser/compiler path. Keeping the path flow untouched minimizes regression risk.

Alternatives considered:

- **Write `robot_description` to a temporary URDF file and call `mj_loadXML`:** broadly compatible, but adds filesystem lifecycle concerns and makes error paths harder to reason about.
- **Use MuJoCo VFS for the URDF string:** viable, but unnecessary for the first pass if package URI mesh references are normalized to real paths. VFS can be added later if in-memory mesh resources become necessary.

### 3. Resolve `package://` mesh URIs before giving URDF strings to MuJoCo

URDF generated by xacro commonly uses `package://...` mesh paths. ros2_control and URDF tooling can understand those conventions, but MuJoCo's XML compiler expects file paths or resources it can open. For URDF string loading, preprocess mesh filename attributes that start with `package://` into absolute filesystem paths using `ament_index_cpp::get_package_share_directory`.

This adds an `ament_index_cpp` dependency to the package. If a package cannot be resolved, fail before MuJoCo compilation and report the unresolved URI.

Rationale: without URI normalization, loading from `robot_description` would work only for URDFs without external mesh assets or with already-absolute paths. Resolving package URIs makes the feature usable with normal ROS robot descriptions.

Alternatives considered:

- **Ask users to avoid `package://`:** rejected because it is common ROS practice.
- **Copy meshes into a MuJoCo VFS:** useful for advanced cases, but more complex and not required for normal filesystem-backed ROS packages.

### 4. Reuse the resolved URDF for ros2_control initialization

Change `MujocoRos2Control` so it can accept an already-resolved `robot_description` string from `mujoco_ros2_control_node.cpp`. If the constructor receives a non-empty string, `init()` should use it directly. If it receives an empty string, it can retain the current parameter/topic lookup behavior for backward compatibility.

Rationale: when URDF is the MuJoCo model source, parsing it twice from independent sources risks subtle mismatches. Passing the resolved string down keeps MuJoCo, ros2_control hardware parsing/component import, and `MujocoSystem::init_sim` aligned.

Alternatives considered:

- **Leave `MujocoRos2Control::get_robot_description()` unchanged:** simple, but it duplicates lookup logic and can select a different description than the one used for MuJoCo compilation.
- **Move all robot-description lookup into `MujocoRos2Control`:** rejected because the executable must load `mjModel` before it constructs `mjData` and initializes rendering/cameras.

### 5. Keep compatibility and diagnostics explicit

The existing `mujoco_model_path` parameter remains the highest-priority source. New URDF behavior is activated only when that path is absent or empty. Error messages should include which source was attempted and, for MuJoCo parse/compile failures, the MuJoCo error string.

Rationale: users with tuned MJCF models may depend on MuJoCo-specific features that URDF cannot represent. Path-first behavior avoids surprising those users while enabling URDF-only launches.

Alternatives considered:

- **Prefer `robot_description` whenever present:** rejected because current launch files pass both parameters, and this would silently change existing demos from MJCF to URDF.

## Risks / Trade-offs

- **Risk: MuJoCo URDF support is more limited than MJCF.** -> Mitigation: keep MJCF/MJB as the default when `mujoco_model_path` is provided, document URDF limitations, and surface MuJoCo compiler errors clearly.
- **Risk: `package://` rewriting may miss uncommon XML shapes or escaped attributes.** -> Mitigation: implement URI normalization with XML parsing rather than fragile string replacement, and test at least one xacro-generated URDF containing mesh filenames.
- **Risk: installed MuJoCo headers may not expose the newer string parse API.** -> Mitigation: verify available APIs during implementation; if `mj_parseXMLString` is unavailable for the supported MuJoCo version, fall back to VFS-backed `mj_loadXML` or a scoped temporary file while keeping the same model-source abstraction.
- **Risk: users expect `/robot_description` to be a global parameter.** -> Mitigation: document that `robot_description` must be provided to `mujoco_ros2_control_node` as a node parameter or published on the supported topic fallback.
- **Risk: URDF and MuJoCo joint naming may diverge when MuJoCo compiles the URDF.** -> Mitigation: retain existing `MujocoSystem::register_joints` validation against MuJoCo joint names and report missing joints with actionable messages.

## Migration Plan

1. Add helper functions for safe optional string parameter reads and model-source resolution in `mujoco_ros2_control_node.cpp` or a small internal utility file.
2. Add URDF string preprocessing for `package://` mesh references and add the required `ament_index_cpp` dependency.
3. Implement URDF string compilation through MuJoCo while preserving the existing `.xml`/`.mjb` path branches.
4. Update `MujocoRos2Control` to accept an optional resolved URDF string and use it before falling back to its current lookup behavior.
5. Add or update a launch/demo path that omits `mujoco_model_path` and relies on `robot_description`.
6. Validate with existing XML/MJB demos and one URDF-only launch.

Rollback is straightforward: keep the old `mujoco_model_path` branch intact and disable the URDF-string branch by requiring `mujoco_model_path` again if compilation or package URI handling proves unstable.

## Open Questions

- Which minimum MuJoCo version should the package declare for `mj_parseXMLString` and `mj_compile` support?
- Should topic fallback for `robot_description` remain indefinite as it is today, or should the new model-source resolver use a timeout to avoid hanging startup?
- Should documentation present URDF loading as the preferred default, or as an additional convenience while MJCF remains recommended for advanced MuJoCo-specific modeling?
