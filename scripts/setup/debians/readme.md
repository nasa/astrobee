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

# Building for the robot (armhf)

Note 1. In order to create the required chroot you must have access to the
astrobee server (it requires VPN).

Note 2. These steps assume you have access and have checkout both the astrobee and
astrobee_platform repositories.

## Environment

Select a directory where to install the Astrobee debbuild chroot

    B=<your directory>
    mkdir -p $B

Define the location of the astrobee and astrobee_platform repository

    P=<astrobee platform directory>
    S=<astrobee FSW src directory>


Select which distribution you want to build for:

    # xenial for Ubuntu 16 or focal for Ubuntu 20
    D=<distribution>

## Create a debbuild chroot

    $P/rootfs/make_chroot.sh $D debbuild $B/debbuild_${D}

## Copy our debian scripts to your chroot

    sudo cp -a $S/scripts/setup/debians $B/debbuild_${D}/data/

## Test your chroot

Access the chroot shell

    sudo $P/rootfs/chroot.sh $B/debbuild_${D}

From the chroot shell, ensure Internet access

    wget www.github.com --no-check-certificate

You should now have an index.html file in the current directory.

Note: In case of failure please review your chroot and host computer DNS settings.
For example, you may want to try editing the /etc/resolv.conf file on your chroot:

    sudo vim $B/debbuild_${D}/etc/resolv.conf
    # You can try moving the 8.8.8.8 server to the top of the file

## Build debians

From your chroot shell, execute the following script to build all debians.
This will also install them in order to test them. You may omit the `--install`
flag if desired.

    cd /data/debians
    ./build_debians.sh --install

Inspect generated files:

    ls $B/debbuild_${D}/data/debians

## Build patched Ubuntu 20 debians

Some Ubuntu packages (fastboot, boringssl, cmake) had to be recompiled 
for the armhf platform we use.

Same as all previous packages, these are already in our repo and only
need to be recompiled if more changes are needed.

To recompile these debians you may execute the scripts under `ubuntu20_patches`
from the same chroot you just used. See the readme for more info.