---
layout: page
title: Installation
permalink: /install/
nav: 2 
parent: Getting started 
---
<details open markdown="block">
<summary>
Table of contents
</summary>
{: .text-delta }
1. TOC
{:toc}
</details>


## Installing from source 
The installation requires [CMake](https://cmake.org/) and [GCC](https://gcc.gnu.org/).

In the path where you want to keep the source code, run the following commands

```shell
git clone https://github.com/darnstrom/daqp.git
mkdir build
cd build
cmake ..
cmake --build .
```


To copy the header files and libraries into `includedir` and `libdir`, respectively, run the following command in the `build` folder 
```shell
cmake --build . --target install
```

### For Windows
The commands mentioned above can be executed on Windows by, for example, using [Git BASH](https://gitforwindows.org/) after installing [CMake](https://cmake.org/) and [TDM-GCC](https://jmeubank.github.io/tdm-gcc/download/). Make sure that CMake is added to PATH during installation.) 


## Installing the MATLAB interface
The MATLAB interface can be installed directly in MATLAB by running the following commands in the directory where you want to store the m-files:
```shell
websave('install_daqp','https://raw.githubusercontent.com/darnstrom/daqp/master/interfaces/daqp-matlab/install_daqp.m')
install_daqp
```
It is also possible to generate the mex-file used in the MATLAB interface when building from source by passing the flag `MATLAB` when setting up CMake, or by running the script `make_daqp` in MATLAB.

## Installing the Julia interface
In the REPL run the command 
```julia
] add DAQP 
```

## Installing the Python interface
Move to the `daqp-python` subdirectory and call pip from the shell:
```shell
pip install .
```
