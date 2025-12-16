# BLASFEO - BLAS For Embedded Optimization

BLASFEO provides a set of basic linear algebra routines, performance-optimized for matrices that fit in cache (i.e. generally up to a couple hundred size in each dimension), as typically encountered in embedded optimization applications.

## BLASFEO APIs

BLASFEO provides two APIs (Application Programming Interfaces):
- BLAS API: the standard BLAS and LAPACK APIs, with matrices stored in column-major.
- BLASFEO API: BLASFEO's own API is optimized to reduce overhead for small matrices.
It employes structures to describe matrices (```blasfeo_dmat```) and vectors (```blasfeo_dvec```), defined in ```include/blasfeo_common.h```.
The actual implementation of ```blasfeo_dmat``` and ```blasfeo_dvec``` depends on the ```TARGET```, ```LA``` (Linear Algebra) and ```MF``` (Matrix Format) choice.
The API is non-destructive, and compared to the BLAS API it has an additional matrix/vector argument reserved for the output.

| API                           | Optimized (level 3) routines                       |
| ----------------------------- | -------------------------------------------------- |
| BLASFEO <br> (small matrices) | dgemm, dsyrk, dsyr2k, dtrmm, dtrsm, dpotrf, dgetrf, dgeqrf, dgelqf, <br> sgemm, ssyrk, strmm, strsm, spotrf |
| BLAS    <br> (small matrices) | dgemm, dsyrk, dsyr2k, dtrmm, dtrsm, dpotrf, dgetrf <br> sgemm, strsm, spotrf |
| BLAS    <br> (large matrices) | dgemm, dsyrk, dsyr2k, dtrmm*, dtrsm*, dpotrf*, <br> sgemm |

Note: BLASFEO is currently under active development.
Some of the routines listed in the previous table may only be optimized for some variants, and provide reference implementations for other variants.
E.g. only some variants of the routines marked with '*' are optimized for large matrices.

## Supported Computer Architectures

The architecture for BLASFEO to use is specified using the ```TARGET``` build variable.
Currently BLASFEO supports the following architectures:

| TARGET                       | Description |
| ---------------------------- | ------------------------------------------------------------- |
| ```X64_INTEL_SKYLAKE_X```    | Intel Skylake-X architecture or newer (optimized for 2 512-bit FMA pipes). x86_64 with AVX512 (F+VL) ISA, 64-bit OS |
| ```X64_INTEL_HASWELL```      | Intel Haswell, Intel Skylake, AMD Zen, AMD Zen2, AMD Zen3 architectures or newer. x86_64 with AVX2 and FMA ISA, 64-bit OS |
| ```X64_INTEL_SANDY_BRIDGE``` | Intel Sandy-Bridge architecture. x86_64 with AVX ISA, 64-bit OS |
| ```X64_INTEL_CORE```         | Intel Core architecture. x86_64 with SSE3 ISA, 64-bit OS |
| ```X64_AMD_BULLDOZER```      | AMD Bulldozer architecture. x86_64 with AVX and FMA ISAs, 64-bit OS |
| ```X86_AMD_JAGUAR```         | AMD Jaguar architecture. x86 with AVX ISA, 32-bit OS |
| ```X86_AMD_BARCELONA```      | AMD Barcelona architecture. x86 with SSE3 ISA, 32-bit OS |
| ```ARMV8A_APPLE_M1```        | Apple M1 architecture or newer. ARMv8A with VFPv4 and NEONv2 ISAs, 64-bit OS |
| ```ARMV8A_ARM_CORTEX_A76```  | ARM Cortex A76 architecture or newer. ARMv8A with VFPv4 and NEONv2 ISAs, 64-bit OS |
| ```ARMV8A_ARM_CORTEX_A73```  | ARM Cortex A73 architecture or newer. ARMv8A with VFPv4 and NEONv2 ISAs, 64-bit OS |
| ```ARMV8A_ARM_CORTEX_A57```  | ARM Cortex A57, A72 architectures. ARMv8A with VFPv4 and NEONv2 ISAs, 64-bit OS |
| ```ARMV8A_ARM_CORTEX_A55```  | ARM Cortex A55 architecture. ARMv8A with VFPv4 and NEONv2 ISAs, 64-bit OS |
| ```ARMV8A_ARM_CORTEX_A53```  | ARM Cortex A53 architecture. ARMv8A with VFPv4 and NEONv2 ISAs, 64-bit OS |
| ```ARMV7A_ARM_CORTEX_A15```  | ARM Cortex A15 architecture. ARMv7A with VFPv4 and NEON ISAs, 32-bit OS |
| ```ARMV7A_ARM_CORTEX_A9```   | ARM Cortex A9 architecture. ARMv7A with VFPv3 and NEON ISAs, 32-bit OS |
| ```ARMV7A_ARM_CORTEX_A7```   | ARM Cortex A7 architecture. ARMv7A with VFPv4 and NEON ISAs, 32-bit OS |
| ```GENERIC```                | Generic target, coded in C, giving better performance if the architecture provides more than 16 scalar FP registers (e.g. many RISC such as ARM) |

Note that the ```X64_INTEL_SKYLAKE_X```, ```ARMV8A_APPLE_M1```, ```ARMV8A_ARM_CORTEX_A76```, ```ARMV8A_ARM_CORTEX_A73```, ```ARMV8A_ARM_CORTEX_A55```, ```X86_AMD_JAGUAR``` and ```X86_AMD_BARCELONA``` architectures are not currently supported by the CMake build system and can only be used through the included Makefile.

### Automatic Target Detection

When using the CMake build system, it is possible to automatically detect the X64 target the current computer can use.
This can be enabled by specifying the ```X64_AUTOMATIC``` target.
In this mode, the build system will automatically search through the X64 targets to find the best one that can both compile and run on the host machine.

### Target Testing

When using the CMake build system, tests will automatically be performed to see if the current compiler can compile the needed code for the selected target and that the current computer can execute the code compiled for the current target.
The execution test can be disabled by setting the ```BLASFEO_CROSSCOMPILING``` flag to true.
This is automatically done when CMake detects that cross compilation is happening.

## Linear Algebra Routines

The BLASFEO backend provides three possible implementations of each linear algebra routine, specified using the ```LA``` build variable:

| LA                          | Description |
| --------------------------- | ------------------------------------------------------------- |
| ```HIGH_PERFORMANCE```      | Target-tailored; performance-optimized for cache resident matrices; panel- or column-major matrix format. Currently provided for OS_LINUX (x86_64 64-bit, x86 32-bit, ARMv8A 64-bit, ARMv7A 32-bit), OS_WINDOWS (x86_64 64-bit) and OS_MAC (x86_64 64-bit). |
| ```REFERENCE```             | Target-unspecific lightly-optimizated; small code footprint; panel- or column-major matrix format |
| ```EXTERNAL_BLAS_WRAPPER``` | Call to external BLAS and LAPACK libraries; column-major matrix format |

## Matrix Formats

Currently there are two matrix formats used in the BLASFEO matrix structures ```blasfeo_dmat``` and ```blasfeo_smat```, specified using the ```MF``` build variable:
| MF             | Description |
| -------------- | ----------- |
| ```COLMAJ```   | column-major (or FORTRAN-style): the standard matrix format used in the BLAS and LAPACK libraries |
| ```PANELMAJ``` | panel-major: BLASFEO's own matrix format, which is designed to improve performance for matrices fitting in cache. Each matrix is stored in block-row-major with blocks (called panels) of fixed height, and within each panel the matrix elements are stored in column-major. |

## Tests

BLASFEO provides some functionality to test the correctness of its linear algebra routines, for both the BLASFEO and the BLAS APIs.
The testing framework is written in python (minimum version 3.6) and uses ```jinja``` template engine, which can be installed with the command ```pip install jinja2```.
In the ```tests``` folder there are several predefined test sets targeting different combinations of architecture, precision and matrix format, and which are used for automatic testing in Travis CI.

In order to run a test set, from the `tests` folder run for example the command 

```python tester.py testset_travis_blasfeo_pm_double_amd64.json```

where you can replace the testset with any other.
If no test set is specified, the ```testset_default.json``` is selected; this testset can be easily edited to test just a few routines of your choice.

## Recommended guidelines

Some general guidelines to install BLASFEO, maximise its performance and avoid known performance issues can be found in the file
[guidelines.md](https://github.com/giaf/blasfeo/blob/master/guidelines.md). <br/>
Covered topics:
- installation tips on Android
- denormals
- memory alignment

## More Information

More information can be found on the BLASFEO wiki at https://blasfeo.syscop.de, including more detailed installation instructions, examples, and a rich collection of benchmarks and comparisions.

More scientific information can be found in:
- the original BLASFEO paper describes the BLASFEO API and the backend (comprising the panel-major matrix format): <br/>
G. Frison, D. Kouzoupis, T. Sartor, A. Zanelli, M. Diehl, *BLASFEO: basic linear algebra subroutines for embedded optimization*. ACM Transactions on Mathematical Software (TOMS), 2018. <br/>
(arXiv preprint https://arxiv.org/abs/1704.02457 )
- the second BLASFEO paper describes the BLAS API implementation and the assembly framework with its custom function calling convention: <br/>
G. Frison, T. Sartor, A. Zanelli, M. Diehl, *The BLAS API of BLASFEO: optimizing performance for small matrices*, 2019. <br/>
(arXiv preprint https://arxiv.org/abs/1902.08115 )
- slides introducing BLASFEO (presented at the 2017 BLIS retreat): <br/>
www.cs.utexas.edu/users/flame/BLISRetreat2017/slides/Gianluca_BLIS_Retreat_2017.pdf
- video with comments to the slides: <br/>
https://utexas.app.box.com/s/yt2d693v8xc37yyjklnf4a4y1ldvyzon
- 3-minutes video on packing strategies in BLAS API (presented at the 2020 BLIS retreat): <br/>
https://www.cs.utexas.edu/users/flame/BLISRetreat2020/Gianlucca.html

## Notes

- BLASFEO is released under the 2-Clause BSD License.

- 06-01-2018: BLASFEO employs now a new naming convention.
The bash script change_name.sh can be used to automatically change the source code of any software using BLASFEO to adapt it to the new naming convention.
