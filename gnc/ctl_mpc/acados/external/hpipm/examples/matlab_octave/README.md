HPIPM examples using the mex-based Matlab/Octave interface.

In order to be able to run the examples in this folder, it is necessary to compile the shared libraries libblasfeo.so and libhpipm.so.
The required shared libraries can be compiled by running
```
make shared_library -j 4
```
(and optionally installed by running `make install_shared`) from the blasfeo and hpipm root folders.

Afterwards, the mex interface has to be compiled.
In order to do so, open a terminal and navigate to the folder `hpipm/interfaces/matlab_octave`, then type the command
```
source env.sh
```
This will set some environment flags (comprising LD_LIBRARY_PATH).
In doing so, it is necessary to know the location of the blasfeo and hpipm root folders (or alternatively, installation folders), which can be specified exporting the environment flags `HPIPM_MAIN_FOLDER` and `BLASFEO_MAIN_FOLDER` _before_ executing the command `source env.sh`.
If not specified, default locations relataive to the current examples folder will be used (see the `end.sh` file for more details.
Afterwards, open Matlab or Octave from the same terminal and compile the interface by running the function `compile_mex_all` (this step needs to be repeated only if the BLASFEO or HPIPM C libraries are recompiled).

Finally, to run the examples, open a terminal and navigate to the current examples folder, then type the command `source env.sh`.
Afterwards, open Matlab or Octave from the same terminal and run the examples.

