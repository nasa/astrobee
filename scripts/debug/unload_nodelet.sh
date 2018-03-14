#!/bin/bash

nodelet_name=$1

if [ "$nodelet_name" = executive ]
then
  manager_name="mlp_management"
elif [ "$nodelet_name" = sys_monitor ]
then
  manager_name="mlp_monitors"
elif [ "$nodelet_name" = llp_cpu_monitor ]
then
  manager_name="llp_monitors"
elif [ "$nodelet_name" = mlp_cpu_monitor ]
then
  manager_name="mlp_monitors"
elif [ "$nodelet_name" = llp_disk_monitor ]
then
  manager_name="llp_monitors"
elif [ "$nodelet_name" = mlp_disk_monitor ]
then
  manager_name="mlp_monitors"
elif [ "$nodelet_name" = speed_cam ]
then
  manager_name="llp_serial"
elif [ "$nodelet_name" = pico_driver ]
then
  manager_name="mlp_depth_cam"
elif [ "$nodelet_name" = choreographer ]
then
  manager_name="mlp_mobility"
elif [ "$nodelet_name" = mapper ]
then
  manager_name="mlp_mobility"
fi

rosservice call /"$manager_name"/unload_nodelet /"$nodelet_name"
