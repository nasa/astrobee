#!/bin/bash

nodelet_name=$1

if [ "$nodelet_name" = executive ]
then
  manager_name="mlp_management"
  nodelet_type="executive/Executive"
elif [ "$nodelet_name" = sys_monitor ]
then
  manager_name="mlp_monitors"
  nodelet_type="sys_monitor/SysMonitor"
elif [ "$nodelet_name" = llp_cpu_monitor ]
then
  manager_name="llp_monitors"
  nodelet_type="cpu_monitor/CpuMonitor"
elif [ "$nodelet_name" = mlp_cpu_monitor ]
then
  manager_name="mlp_monitors"
  nodelet_type="cpu_monitor/CpuMonitor"
elif [ "$nodelet_name" = llp_disk_monitor ]
then
  manager_name="llp_monitors"
  nodelet_type="disk_monitor/DiskMonitor"
elif [ "$nodelet_name" = mlp_disk_monitor ]
then
  manager_name="mlp_monitors"
  nodelet_type="disk_monitor/DiskMonitor"
elif [ "$nodelet_name" = speed_cam ]
then
  manager_name="llp_serial"
  nodelet_type="speed_cam/SpeedCamNode"
elif [ "$nodelet_name" = pico_driver ]
then
  manager_name="mlp_depth_cam"
  nodelet_type="pico_driver/PicoDriverNodelet"
elif [ "$nodelet_name" = choreographer ]
then
  manager_name="mlp_mobility"
  nodelet_type="choreographer/ChoreographerNodelet"
elif [ "$nodelet_name" = sentinel ]
then
  manager_name="mlp_depth_cam"
  nodelet_type="sentinel/SentinelNodelet"
fi

rosservice call /"$manager_name"/load_nodelet /"$nodelet_name" "$nodelet_type" [] [] [] ""
