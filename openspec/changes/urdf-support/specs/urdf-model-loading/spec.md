## ADDED Requirements

### Requirement: Preserve existing MuJoCo model path loading
The system SHALL continue to load `mujoco_model_path` when it is provided and non-empty, using binary model loading for `.mjb` files and XML model loading for other paths.

#### Scenario: Existing XML path is provided
- **WHEN** the node starts with a non-empty `mujoco_model_path` that references a MuJoCo XML model
- **THEN** the system SHALL load that path using MuJoCo XML loading and SHALL NOT replace it with `robot_description`

#### Scenario: Existing MJB path is provided
- **WHEN** the node starts with a non-empty `mujoco_model_path` ending in `.mjb`
- **THEN** the system SHALL load that path using MuJoCo binary model loading

### Requirement: Load MuJoCo model from robot_description when no model path is provided
The system SHALL support startup without `mujoco_model_path` by resolving URDF content from `robot_description` and compiling it into a MuJoCo model before creating simulation data.

#### Scenario: robot_description parameter is available
- **WHEN** the node starts without a non-empty `mujoco_model_path` and has a non-empty `robot_description` parameter
- **THEN** the system SHALL compile the `robot_description` URDF into a MuJoCo model

#### Scenario: robot_description parameter is empty or absent
- **WHEN** the node starts without a non-empty `mujoco_model_path` and cannot resolve non-empty `robot_description` content
- **THEN** the system SHALL fail initialization with an error that names `mujoco_model_path` and `robot_description` as accepted model sources

### Requirement: Resolve robot_description from the running node context
The system SHALL resolve `robot_description` from the existing `mujoco_ros2_control` node context rather than relying on an unrelated temporary node as the primary parameter source.

#### Scenario: robot_description is passed to mujoco_ros2_control node
- **WHEN** launch configuration provides `robot_description` in the `mujoco_ros2_control` node parameters
- **THEN** the system SHALL read that parameter from the same node context used for model initialization

#### Scenario: temporary node would not receive the parameter override
- **WHEN** no `robot_description` value is available in the running node context
- **THEN** the system SHALL NOT report success based solely on creating a separate temporary node

### Requirement: Reuse one resolved URDF for MuJoCo and ros2_control
The system SHALL use the same resolved URDF content for MuJoCo model compilation and ros2_control resource initialization when URDF loading is selected.

#### Scenario: URDF loading succeeds
- **WHEN** MuJoCo model compilation is performed from resolved `robot_description` content
- **THEN** the system SHALL pass that same URDF content to ros2_control parsing and resource manager initialization

#### Scenario: URDF content changes between independent sources
- **WHEN** another potential `robot_description` source differs from the content used for MuJoCo compilation
- **THEN** the system SHALL keep ros2_control initialization aligned with the content used for MuJoCo compilation

### Requirement: Normalize ROS package mesh URIs for MuJoCo
The system SHALL handle URDF mesh filenames that use `package://` URIs by resolving them to filesystem paths before MuJoCo compilation or by otherwise providing equivalent resolvable assets to MuJoCo.

#### Scenario: package URI mesh can be resolved
- **WHEN** `robot_description` contains a mesh filename with a resolvable `package://` URI
- **THEN** the system SHALL provide MuJoCo with a resolvable asset path for that mesh

#### Scenario: package URI mesh cannot be resolved
- **WHEN** `robot_description` contains a mesh filename with an unresolvable `package://` URI
- **THEN** the system SHALL fail initialization with an error that identifies the unresolved URI

### Requirement: Report MuJoCo URDF compilation failures clearly
The system SHALL surface MuJoCo parse or compile errors when URDF-based model loading fails.

#### Scenario: MuJoCo rejects the URDF
- **WHEN** the resolved `robot_description` content cannot be parsed or compiled by MuJoCo
- **THEN** the system SHALL fail initialization and include the MuJoCo error details in the log output

#### Scenario: MuJoCo compiles the URDF
- **WHEN** the resolved `robot_description` content is accepted by MuJoCo
- **THEN** the system SHALL create simulation data from the compiled model and continue normal controller initialization

### Requirement: Keep model loading outside the control loop
The system SHALL perform URDF resolution, asset normalization, and MuJoCo model compilation during startup before the control update loop begins.

#### Scenario: Control loop is active
- **WHEN** controller manager read, update, or write callbacks are running
- **THEN** the system SHALL NOT perform URDF parsing, package URI resolution, or MuJoCo model compilation as part of those callbacks
