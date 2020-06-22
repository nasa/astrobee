\page urdf Description

The Universal Robot Description Format (URDF) is a language for describing a robot in terms of its links and joints. It is used by rviz to render robots, robot_state_publisher to convert joint states to TF2 transforms, and by Gazebo to simulate a robot. Xacro is a scripting environment for generating URDF files, which allows one to pass parameters from launch files to the URDF, as well as create more modular functions and snippets that can be incorporated into a single larger URDF.

This folder contains only the Xacro files for the free-flyer model. The dock, ISS and granite models are static URDFs that are included in the media distribution. This design choice enables us to load these entities from a world file, preventing race conditions in loading environmental elements with free-flyers in a simulated context.

The way URDF works is best illustrated by this example, which is actually loosely representative of how the project's launch files actually work:

    <arg name="ns" value="honey"/>
    <group ns="$(arg ns)">
      <param name="robot_description" command="$(find xacro)/xacro --inorder $(find description)/urdf/astrobee.urdf.xacro" />
      <node name="spawn_astrobee" pkg="astrobee_gazebo" type="spawn_model"
            args="-param robot_description -urdf -model $(arg ns)" />
      <node pkg="robot_state_publisher" type="robot_state_publisher"
            name="astrobee_state_publisher" />
    </group>

Line (1) specifies the namespace on which the robot will operate.
Line (2) creates the namespace.
Line (3) invokes xacro to write URDF to parameter /honey/robot_description.
Line (4) spawns a simulated and namespaced astrobee using the URDF parameter.
Line (5) starts robot state publisher to map joint states to transforms.

There is a lot of information encoded in Line 5, and so it pays to expand upon what it does. Essentially it starts a robot_state_publisher node that listens to /joint_states, and with a knowledge of the robot description at URDF parameter /honey/robot_description produces TF2 transforms for all links. This is an extremely useful service, as rviz is only able to render models for which transforms exist, and we need some entity to produce those transforms.
