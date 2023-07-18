HPIPM examples using the hpipm_python module.

In order to be able to run the examples in this folder, it is necessary to have Python 3 installed in the system.
The first step is to install the shared libraries libblasfeo.so and libhpipm.so and to run
```
pip install .
```
from hpipm/interface/python/hpipm_python (depending on your system, you may need to run pip3 instead).

The required shared libraries can be installed by running 
```
make shared_library -j4 & sudo make install_shared
```
from the blasfeo and hpipm root folder.
Make sure that the location of libblasfeo.so and libhpipm.so is known to the system by adding /opt/blasfeo/lib and /opt/hpipm/lib to LD_LIBRARY_PATH as
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/blasfeo/lib:/opt/hpipm/lib
```
