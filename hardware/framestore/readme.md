\defgroup framestore Frame Store
\ingroup hw

The `framestore` node broadcasts a set of transforms to /tf_static, which enables easy transforms to and from coordinte frames. The frames are listed in `hw/framestore.config` and are prefixed by the platform name to avoid any naming collisions with multiple astrobees.