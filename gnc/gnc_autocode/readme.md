\defgroup gncautocode GNC Autocode Wrapper
\ingroup gnc

A thin wrapper around the C code automatically generated from Matlab and Simulink.

Each subsystem of GNC has a class wrapping the functionality, which manages
the underlying Simulink structs. Each class calls the proper allocation and
deallocation functions in the constructor and destructor. Then two functions
capture the rest of the functionality:

* `Initialize`: This initializes the underlying objects. If called more than once,
the objects are reset to their initial state.
* `Step`: This runs the subsystem one step forward in time. It may takes
inputs from the preceding subsystem.

