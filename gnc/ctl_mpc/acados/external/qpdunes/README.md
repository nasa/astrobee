qpDUNES-dev
===========

Private development repository for qpDUNES



+=================================================================+

INSTALLATION:
-------------

1) build the qpDUNES source code from the qpDUNES root directory:

    cd [qpDUNES-dir]
    make


SOLVE A QP:
-----------

1) go to examples directory:
    
    cd [qpDUNES-dir]/examples

2) build the examples:

    make

3) run an example of your choice

    ./example1

INSTALLATION WITH CMAKE:
-------------

```
mkdir build
cd build
cmake -DQPDUNES_SIMPLE_BOUNDS_ONLY:BOOL=OFF
make
make install
```

after this you can run test, optionally:
```
make test
```

+=================================================================+