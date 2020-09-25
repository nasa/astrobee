\page mem_monitor Memory Monitor

The memory monitor node is responsible for reporting memory useage information. One instance will run on the LLP and another will run on the MLP. It reports the loads and frequency of each cpu and the temperature of the cpus in a cpu state message. Furthermore, it asserts faults if the temperature or average cpu load gets to high.
