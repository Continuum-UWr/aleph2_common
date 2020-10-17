# aleph2_description
This package provides a URDF model of the `Aleph2` robot that describes its visual, kinematic and physical properties. It uses [xacro] to make the description more human-readable, by using macros, math expressions, conditional blocks and by splitting the description into multiple files.

## Naming conventions
To make sure the names across the model are consistent, a set of naming conventions for links and joints has been established:

  - Root link of the robot should be named `base_link`.
  - Link names should end with a `_link` keyword if the link has any visual, collision or physical properties.
  - If the link does not contain any properties and is only used as a reference frame, its name should contain a `_frame` suffix.
  - Joint names should start with the name of the joint’s child link (without the suffix) and end with a `_joint` keyword.
  - If the joint mimics other joint (contains mimic tag), its name should end with `_mimic_joint` suffix.

## Launch files

* **`description.launch`** 
 
    Sets the `robot_description` parameter on the [Parameter Server] to the URDF description of the robot.

    **Arguments:**
    * `model` (default: `aleph2_description/urdf/aleph2.urdf.xacro`)
    
        Path to the file containing the robot description in URDF format.

* **`state_publisher.launch`**

    Includes `description.launch`. \
    Starts [robot_state_publisher] which publishes the state of the robot to [tf2].

[xacro]: http://wiki.ros.org/xacro
[Parameter Server]: http://wiki.ros.org/Parameter%20Server
[robot_state_publisher]: http://wiki.ros.org/robot_state_publisher
[tf2]: https://wiki.ros.org/tf2