\page cpu_monitor CPU Monitor

The cpu monitor node is responsible for reporting cpu information. One instance will run on the LLP and another will run on the MLP. It reports the loads and frequency of each cpu and the temperature of the cpus in a cpu state message. Furthermore, it asserts faults if the temperature or average cpu load gets to high.
