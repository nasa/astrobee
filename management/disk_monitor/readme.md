\defgroup disk_monitor Disk Monitor
\ingroup management

The disk monitor node is responsible for reporting disk information. One instance will run on the LLP and another will run on the MLP. It uses the file pathnames specified in the disk\_monitor.config file to retrive information on the filesystems that contain those files. The capacities and the amounts used are sent out in a disk state message.
