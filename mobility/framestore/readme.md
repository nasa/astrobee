\defgroup framestore Framestore
\ingroup mobility

We can broadly divide all static transforms into *global* and *local* types based on its highest rigid ancestor. Intuitively, if an entity is fixed with respect to the world, then it's a global static transform. Conversely, if the entity is fixed with respect to a robot, then it's a local static transform. For example, the IMU and dock poses are local and global respectively.

In a real experiment an Astrobee is launched without a simulator or any other external computer, so we cannot rely on the simulation (or some other computer) to broadcast the global transforms. Similarly, we can launch a simulation without a Free Flyer, and we should expect rviz to position the ISS and dock correctly. So, we also cannot rely on the presence of a Free Flyer to obtain global transforms. The solution is to duplicate our global transforms -- this keeps simplicity at the cost of increasing the number of messages.

The framestore nodelet is responsible for reading all transforms from the LUA transforms.config and broadcasting them as static transforms on TF2, where they may be picked up by other ROS nodes including the Gazebo simulator plugins, which use the transforms to reposition sensors and simulated entities.

The global_transform node performs a similar function, but only broadcasts the global transforms, so that in the absence of a Free Flyer the graphical user interfaces and simulator show the ISS and dock in the correct location.
