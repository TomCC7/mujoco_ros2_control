How To Setup mujoco_ros2_control
================================

Introduction
------------

This document provides guidance on how to setup and use ``mujoco_ros2_control`` package.
This page will demonstrate the supported MuJoCo simulation features with a robot manipulator.
However, this package is not specifically designed for manipulation and can be used for any types of robots.


Prerequisites
--------------

Running on the local machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you want to run an application on the local machine, you need following dependencies installed on the machine.

1. `MuJoCo <https://github.com/google-deepmind/mujoco>`_
2. `ROS 2 humble <https://docs.ros.org/en/humble/Installation.html>`_
3. `MoveIt 2 <https://github.com/moveit/moveit2>`_


Installation
------------

Running on the local machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Create a workspace
  .. code-block:: bash

    mkdir -p ~/mujoco_ros_ws/src

2. Clone the repository.
  .. code-block:: bash

    cd ~/mujoco_ros_ws/src
    git clone https://github.com/sangteak601/mujoco_ros2_control.git

3. Set environment variable
  .. code-block:: bash

    export MUJOCO_DIR=/PATH/TO/MUJOCO/mujoco-3.x.x

4. Build the package
  .. code-block:: bash

    sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ~/mujoco_ros_ws
    colcon build


How to Setup mujoco_ros2_control
--------------------------------

.. note:: Please refer to `this <https://github.com/sangteak601/mujoco_ros2_control/blob/moveit_doc/doc/index.rst#usage>`_ for the details.

Set ros2_control plugin in the URDF
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can configure the `ros2_control` hardware system plugin for MuJoCo in the URDF file as follows:

.. code-block:: XML

  <ros2_control name="MujocoSystem" type="system">
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    </hardware>
  </ros2_control>

You can also set parameters for the plugin such as pid gains, min/max effort and so on.
To find examples of parameters, please see `urdf examples <https://github.com/sangteak601/mujoco_ros2_control/tree/moveit_doc/mujoco_ros2_control_demos/urdf>`_.

Choose URDF, MJCF, or MJB model input
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can pass a URDF through the node-scoped ``robot_description`` parameter, or
you can pass a MJCF/MJB file through ``mujoco_model_path``. MJCF/MJB remains the
best option when the model needs MuJoCo-specific features that URDF cannot
express directly. When you do convert URDF to MJCF, make sure to use the
**same name** for the ``link`` and ``joint``, which are mapped to the ``body``
and ``joint`` in MuJoCo. You can specify position limits in ``<limit>`` in MJCF,
and effort limits in URDF as shown in this
`example <https://github.com/sangteak601/mujoco_ros2_control/blob/moveit_doc/mujoco_ros2_control_demos/urdf/test_cart_effort.xacro.urdf>`_.
Velocity limits will not be applied at all.

Any force torque sensors need to be mapped to separate force and torque sensors in the MJCF, since there is no support for combined sensors in MuJoCo.
The name of each sensor should be sensor_name + _force and sensor_name + _torque.
For example, if you have a force torque sensor called ``my_sensor``, you need to create ``my_sensor_force`` and ``my_sensor_torque`` in MJCF.

Check `mujoco_models <https://github.com/sangteak601/mujoco_ros2_control/tree/moveit_doc/mujoco_ros2_control_demos/mujoco_models>`_ for examples.

Specify the model source and controller config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can pass the path to MJCF as ``mujoco_model_path`` parameter to the node.
If ``mujoco_model_path`` is omitted or empty, ``mujoco_ros2_control`` reads
``robot_description`` from its own node parameters and asks MuJoCo to compile
that URDF directly. ROS 2 parameters are node-scoped, so provide
``robot_description`` to this node even if another node also receives it.
You also need to pass controller configuration since ``mujoco_ros2_control`` is replacing ``ros2_control`` node.

.. code-block:: Python

  controller_config_file = os.path.join(mujoco_ros2_control_demos_path, 'config', 'cartpole_controller_position.yaml')

  node_mujoco_ros2_control = Node(
      package='mujoco_ros2_control',
      executable='mujoco_ros2_control',
      output='screen',
      parameters=[
          robot_description,
          controller_config_file,
          {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'test_cart_position.xml')}
      ]
  )


Running the MoveIt Interactive Marker Demo with MuJoCo
------------------------------------------------------

.. note:: Please refer to `this <https://github.com/sangteak601/mujoco_ros2_control_examples/tree/main/mujoco_panda>`_ for running the demo.
