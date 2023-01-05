# Astrobee armhf debian patches for Ubuntu 20

This folder contains scripts and patches needed for flight software
on the Ubuntu 20 armhf platform.

## Cmake

CMake 3.16.3 is broken for the armhf setup we use.

In order to compile our custom debians for the robot we need to patch
CMake using the `build_cmake.sh` script.

The patched debians are already in our repositories. You do not need to
run this script again unless an additional change to CMake is needed.
This issue is fixed in CMake 3.19.0.

## BoringSSL

ADB fails with key errors on start. The issue is not exactly on ADB,
rather on the SSL implementation it depends on.

Use `build_boringssl.sh` to patch and create new debians. This may
produce many debians. We only need the following:

- `android-libboringssl_8.1.0+r23-2ubuntu1+focal1_armhf.deb`
- `android-libboringssl-dev_8.1.0+r23-2ubuntu1+focal1_armhf.deb`

Again, these are already in our repositories. Only rebuild if required.

## Fastboot

Fastboot seems to have issues with large files on Ubuntu 20 armhf.

Use `build_fastboot.sh` to rebuild with 64bit offsets. This may create
many debians. We only need the following:

- `fastboot_8.1.0+r23-5ubuntu2+focal1_armhf.deb`

Yet again, this is already in our repo. Rebuild only if required.

