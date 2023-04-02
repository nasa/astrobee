This example shows how to use the code-generated QP data (see MALTAB/Octave and Python examples) to solve QPs with HPIPM from simulink.
In order to be able to use the S-Function within a Simulink block, copy the C file containing the QP data to the current folder (and change the path in the `make_sfun.m` script).
Then run `make_sfun.m` and `load_parameters.m`.
This will compile the HPIPM S-Function into MEX binaries that you can from the Simulink block `hpipm_simulink_getting_started.slx`.
Notice that the make script will make use of the environment variables usually set by sourcing `env.sh`.
The default `qp_data.c` that you find in this folder refers to the `getting_started` (MATLAB/Octave or Python) example.
When you update the data, you will need to adjust the dimension of the signals in the Simulink block accordingly. 
