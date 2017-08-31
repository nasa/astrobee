\mainpage Astrobee Debians

This folder contains scripts and patches to build debians needed for flight software.

# Organization

- `build_xxx.sh` Each package contains a script which downloads the source and compiles
  the package into a .deb file. To build for the robot, run this script inside the debbuild chroot.
- `xxx/` This is the 'debian' folder used to build the package. It follows the standard
  format documented by debian. A few key details:
  - `patches/` This directory contains all patches, managed with quilt.
  - `changelog` A log of changes. Update this to bump the version number.
  - `control` Description and dependencies list.
  - `xxx.install` A list of the files to install as part of the package.
  - `rules` Script for compiling the code.

# Updating a Package

When updating a package, change the git commit to download in the script. Then bump the version
number with a new entry in the changelog.

If changes to the patches are needed, work in the libxxx directory created by the script.
Use `quilt` to modify the patches.

