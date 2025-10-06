HPIPM examples using the hpipm_matlab package.

In order to be able to run the examples in this folder from Octave, it is necessary to install the shared libraries libblasfeo.so and libhpipm.so.
The required shared libraries can be installed by running
```
make shared_library -j4 & sudo make install_shared
```
from the blasfeo and hpipm root folder.
Make sure that the location of libblasfeo.so and libhpipm.so is known to the system by adding /opt/blasfeo/lib and /opt/hpipm/lib to LD_LIBRARY_PATH as
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/blasfeo/lib:/opt/hpipm/lib
```
To be able to call the Python interpreter from Octave, you will need to install the Pytave package (follow the instructions here https://bitbucket.org/mtmiller/pytave).
During the installation process, before the configuration step you might need to run
```
export PYTHON_VERSION=<version>
```
where your python interpreter binaries name reads `python<version>`.

Finally you will need to make sure that hpipm/interfaces/matlab/hpipm_matlab is in your Octave path and that hpipm_python has been installed for the Python version used by Octave.
You can check which version is used by running the command
```
py.sys.path
```
from the Octave command line.


