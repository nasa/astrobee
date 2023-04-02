HPIPM examples using the hpipm_matlab package.

In order to be able to run the examples in this folder, it is necessary to install the shared libraries libblasfeo.so and libhpipm.so.
The required shared libraries can be installed by running
```
make shared_library -j4 & sudo make install_shared
```
from the blasfeo and hpipm root folder.
Make sure that the location of libblasfeo.so and libhpipm.so is known to the system by adding /opt/blasfeo/lib and /opt/hpipm/lib to LD_LIBRARY_PATH as
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/blasfeo/lib:/opt/hpipm/lib
```
(notice that, under some OSs, the value environment variable LD_LIBRARY_PATH within MATLAB can be different from the system's value. As a quick workaround, try to start MATLAB from a terminal).

Finally you will need to make sure that hpipm/interfaces/matlab/hpipm_matlab is in your MATLAB path and that hpipm_python has been installed for the Python version used by MATLAB.
You can check which version is used by running the command
```
py.sys.path
```
from the MATLAB command line and change it by running
```
pyversion <path_to_the_python_interpreter>
```

