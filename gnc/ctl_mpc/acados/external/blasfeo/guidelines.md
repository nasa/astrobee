## Installation on Android

BLASFEO can successfully run on the Android OS.
The procedure has been tested for the most common combination (namely Android running on the ARMv8A architecture, with the BLASFEO code cross-compiled from a x86_64 Linux host machine), but it is expected to work on other combinations too by using the OS_LINUX architecture in BLASFEO.

In the tested configuration, BLASFEO was compiled with the ```aarch64-linux-android-gcc``` cross-compiler provided by the Android NDK.
Once downloaded and unzipped into ```NDK_MAIN_DIR```, the NDK tools themselves can be installed on ```/opt/ndk``` by using the script ```$(NDK_MAIN_DIR)/build/tools/make-standalone-toolchain.sh``` as <br/>
```./make-standalone-toolchain.sh --arch=arm64 --install_dir=/opt/ndk```

Once the BLASFEO static library has been compiled and the executable ```example.out``` created, this can be moved to the Android device by means of the command <br/>
```adb push example.out /data/local/tmp/example.out``` <br/>
and executed with the command <br/>
```adb shell /data/local/tmp/example.out```


## Performance issues

The performance of BLASFEO routines can be affected by many factor, and some can have a large impact on performance.

Known performance issues:
- computations on __denormals__.
In some computer architectures (like e.g. the widespread x86_64) computations involving denormal floating point numbers are handled in microcode, and therefore can incur in a very large performance penalty (10x or more).
Unless computation on denormals is on purpose, the user should pay attention to avoid denormals on the data matrices as well as to the __memory passed to create BLASFEO matrices or vectors__ (as the padding memory is still used in internal computations, even if it is discarded and does not affect the correctness of the result).
As a good practice, since denormals can be left in the memory by previous applications, it is __recommended to zero out__ the memory passed to create a BLASFEO matrix or vector in the BLASFEO API (i.e. the memory passed to the routines `blasfeo_create_dmat` and similar).
For similar reasons, in the BLAS API it is recommended to have a leading dimension of the matrices multiple of the minimum BLASFEO kernel size (typically equal to 4 in double precision), and to zero out the memory used for the entire array of doubles or floats (including padding).
- __memory alignment__.
The memory passed to create BLASFEO matrices or vectors has minimum alignment requirements which vary between architecture.
Additionally, if the memory is not aligned to cache size boundaries, there may be little performance degradations.
As a good practice, it is __recommended to align to cache line boundaries__ (typically 64 bytes) the memory passed to create a BLASFEO matrix or vector in the BLASFEO API, or the memory used for the array of doubles of floats in the BLAS API.
