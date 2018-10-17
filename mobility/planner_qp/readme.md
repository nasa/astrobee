\defgroup planner_qp Quadratic Program (QP) planner
\ingroup mobility

# Overview 

This planner is based off Michael Watterson's PhD thesis. We generate trajectories by running an optimization which minimizes the functional:

![alt text](../images/mobility/planner_qp_cost.png)

With collision avoidance constraints and scaling to ensure dynamic feasibility.


# Usage

To generate and execute a optimized plan one should use the move command in conjunction with the "qp" planner flag. Here is an example command that shows how to move to the origin of the world:

    rosrun mobility teleop -move -pos "0 0 0" -att "0" -planner qp

By default the teleop tool configures the platform to operate in holomonic mode. To enforce face-forward mode the "-ff" flag can be used. For example:

    rosrun mobility teleop -move -pos "0 0 0" -att "0" -planner qp -ff


# Trajectory Display

This planner comes with an rviz plug-in to view the generated trajectories.  

![alt text](../images/mobility/planner_qp_rviz_1.png)

From the display panel, add the traj_opt_ros trajectory plug-in. 

![alt text](../images/mobility/planner_qp_rviz_2.png)

Change the topic in the plug-in to either the current trajectory or a debug version.  Here the regular trajectory is plotted in magenta and the debug is plotted in red.  The debug shows the optimization at each iteration as it converges to the current trajectory.


# Parameter Description

The planner has adjustable parameters which can affect the optimizer's performance which are configurable through the reconfigure GUI.

    rosrun rqt_reconfigure rqt_reconfigure

![alt text](../images/mobility/planner_qp_config.png)

* `sample_rate` - The optimization uses a continuous trajectory representation which is sub-sampled at this rate before being sent to the choreographer.

* `robot_radius` - How large the robot's center must stay away from obstacles, increase this to create a trajectory further away from walls

* `two_d` - Force the z coordinate of the goal to be the same as the start.  (Mainly for use on the granite table)

* `uniform_time_allocation` -  Make each segment in trajectory spline take the same amount of time.  This usually makes performance worse to enable this.

* `time_optimization` -  Iterates on trajectory spline time allocation. With this option result is better, but takes longer to compute.  This also makes the optimization no longer a quadratic program and non-convex.

* `duality_gap_threshold` -  Main optimization termination criteria, see Bemporad-Morari MPC book for details. Higher is faster, but less optimal. Lower is more optimal, but slower to compute.

* `maximum_iterations` - Limit maximum number of iterations when performing optimization.

* `iteration_replay` - Changes speed of the debug trajectory animation.  Represents the number of seconds for the entire animation.