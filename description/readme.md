\defgroup des Description
\ingroup des

This package provides configuration files and transform utilities that describing the robots and static transforms in our system, providing consistency with or without rviz and Gazebo. More specifically it includes:

1. A collection of URDF files that describe all robots in Astrobee.
2. A global_transform node that broadcasts global frame transforms to correctly position the ISS and dock in Gazebo / rviz in the absence of a Free Flyer.
3. A framestore nodelet that runs on a Free Flyer, broadcasting all local and global frame transforms.

## URDF files

The Universal Robot Description Format (URDF) is a language for describing a robot in terms of its links and joints. It is used by rviz to render robots, robot_state_publisher to convert joint states to TF2 transforms, and by Gazebo to simulate a robot. For simulation the URDF can include <gazebo> tags, which provide simulation-specific instructions like materials and friction.

The URDF files are located in description/urdf and take the *.urdf.xacro suffix. This means that they must be interpreted by xacro, a scripting language to help parameterize what would otherwise be a very restrictive markup language. There are three root URDFs:

* astrobee.urdf.xacro - describes a Free Flyer.
* dock.urdf.xacro - describes a Dock.
* iss.urdf.xacro - describes the International Space Station.

The way URDF works is best illustrated by this hypothetical example, which is actually loosely representative of how the project's launch files actually work:

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

## Global and local static frame transforms

We can broadly divide all static transforms into *global* and *local* types based on its highest rigid ancestor. Intuitively, if an entity is fixed with respect to the world, then it's a global static transform. Conversely, if the entity is fixed with respect to a robot, then it's a local static transform. For example, the IMU and dock poses are local and global respectively.

In a real experiment an Astrobee is launched without a simulator or any other external computer, so we cannot rely on the simulation (or some other computer) to broadcast the global transforms. Similarly, we can launch a simulation without a Free Flyer, and we should expect rviz to position the ISS and dock correctly. So, we also cannot rely on the presence of a Free Flyer to obtain global transforms. The solution is to duplicate our global transforms -- this keeps simplicity at the cost of increasing the number of messages.

The framestore nodelet is responsible for reading all transforms from the LUA transforms.config and broadcasting them as static transforms on TF2, where they may be picked up by other ROS nodes including the Gazebo simulator plugins, which use the transforms to reposition sensors and simulated entities.

The global_transform node performs a similar function, but only broadcasts the global transforms, so that in the absence of a Free Flyer the graphical user interfaces and simulator show the ISS and dock in the correct location.
