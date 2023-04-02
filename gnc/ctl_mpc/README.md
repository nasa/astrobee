# ctl_mpc Package
This is an extension package for the [NASA Astrobee project](https://github.com/nasa/astrobee) which allows MPC with Acados.

## Installation
This assumes you have the astrobee software already installed and built.

1. Clone this repo in a suitable directory, e.g.
```
cd $ASTROBEE_WS/src/gnc
git clone https://github.com/DISCOWER/ctl_acados_mpc.git
cd ctl_acados_mpc
```

2. Now wee need to install acados. First run `git submodule update --recursive --init`

3. Make a build directory and install acados. You can choose which solvers you would like to use with optional flags
```
cd acados
mkdir -p build
cd build
cmake .. # with optional arguments e.g. -DACADOS_WITH_DAQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder>
make install
```
4. In case you want to rebuild the c-code for the MPC (e.g. with a different QP solver, different horizon):
Install the python interface with pip: `pip install -e ./interfaces/acados_template`
Note that some parameters can be changed online (costs, state tolerances, inertia and mass)

5. Add the compiled shared library paths to the following variables:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
export ACADOS_SOURCE_DIR="<acados_root>"
```

6. Build this package by running `catkin build --this --no-deps`

## Usage
1. Open 2 terminals, go to your `$ASTROBEE_WS` and run `source devel/setup.bash` on each

2. In the first terminal, launch the astrobee. E.g. for a simulation, do `roslaunch astrobee astrobee_sim.launch`. Check out the launch files for available settings

3. In the second terminal, you can send the astrobee on a mission! E.g. run: `rosrun executive teleop_tool -move -relative -pos "0 -1 0"`

### (OPTIONAL in case you want to tune params/settings that are unavailable for change in the C++ code)
5. Open `/solver/generate_solver.py` and tune the settings to your liking. Then run `python generate_solver.py`

6. Re-build the package. You can do this by making sure you are in the package root folder (e.g. `$ASTROBEE_WS/gnc/ctl_acados_mpc`) and running `catkin build --this --no-deps`

## Acknowledgements
A special thanks to Pedro Roque for instights, bug-fixing help and code contribution in the CMakeLists.txt.
