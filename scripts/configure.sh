#!/bin/bash
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Helper to configure the astrobee build in a simple a repeatable way
#
# This script simply invoke cmake to configure the build with some
# reasonable options for either Native Linux (-l) or/and ArmHF (-a)
#
# The build path (-b), install path (-p) and build type (-B) have
# default values that can be overriden with the according flags.
# Build path and install path are created by default under the home
# directory.

# First identify more or less where we are (absolute or relative ok)
callcmd=$0
workdir=`pwd`
confdir=`dirname $callcmd`
scriptname=${0##*/}
rootpath=`cd $confdir/..; pwd`

cpu_type=`uname -m`
os_kernel=`uname -s | tr '[:upper:]' '[:lower:]'`
gcc_major=`gcc -dumpversion | cut -f-2 -d .`
archname="${cpu_type}_${os_kernel}_gcc${gcc_major}"

# prefix for installation path
prefix=""

# build name (if used, build under astrobee/build/$buildname)
buildname=""

# configure nothing by default
native_build=0
armhf_build=0

# by default DDS is activated
dds_opt="-DUSE_DDS=on"

# No extra options by default
extra_opts=""

# Force cache cleaning by default
clean_cache=1

# Default debug mode is off
debug_mode=0

# Build target
target="install"

# short help
usage_string="$scriptname [-h] [-l <linux build>] [-a <arm build>]\
 [-p install_path] [-b build_path] [-B build_type] [-c <force clean cache>]\
 [-C <don't clean cache>] [-d <with DDS>] [-D <without DDS>]\
 [-r <with QP planner>] [-R <without QP planner>]\
 [-f <with PicoFlexx driver>] [-F <without PicoFlexx driver>]\
 [-k <use ccache if available>] [-K <do not use ccache>]\
 [-v <with VIVE>] [-V <without VIVE>]\
 [-t <with integration tests, requires gpu>] [-T <without integration test>]\
 [-g <print debug information only>]"
#[-t make_target]

# options to parse
optstring="hlap:b:B:cCdDrRfFkKvVtTg"

# Print the script usage
print_usage()
{
    echo $usage_string
}

# Print the help message (list all the options)
print_help()
{
    echo "scriptname usage:"
    echo $usage_string
    echo -e "\t-l Generate a Native Linux build"
    echo -e "\t-a Generate a cross-compiled ARM build"
    echo -e "\t-p install_path specify the installation directory"
    echo -e "\t   default=${HOME}/cmake_install_platform"
    echo -e "\t-b build_path   specify the build directory to use"
    echo -e "\t   default=${HOME}/cmake_build_platform"
    echo -e "\t-B build_type   specify build type (Debug|Release|RelWithDebInfo)"
    echo -e "\t-c delete the cmake cache before for every modules: default on"
    echo -e "\t   (necessary when re-running buildall with diffent options)"
    echo -e "\t-C do not automatically delete the cmake cache when configure is run"
    echo -e "\t   (need to be specified to avoid cleaning the cache by default)"
    echo -e "\t-k use ccache if available to speed up compilation (default)"
    echo -e "\t-K do not use ccache and turn on native optimizations"
    echo -e "\t-d build with DDS support (default)"
    echo -e "\t-D build without DDS support"
    echo -e "\t-r build the QP planner (default)"
    echo -e "\t-R build without the QP planner"
    echo -e "\t-f build with the PicoFlexx driver (default)"
    echo -e "\t-F build without the PicoFlexx driver"
    echo -e "\t-v build with the VIVE (default)"
    echo -e "\t-V build without VIVE"
    echo -e "\t-t build with the integration tests, if tests enabled, requires gpu (default)"
    echo -e "\t-T build without integration tests, if tests enabled"
    #    echo -e "\t-t make_target  define the make build target"
    #    echo -e "\t   default (when ommited) is 'install'"
    echo -e "\t   when -t is specified, the configure processs is skipped!"
    echo -e "\t-g prints some debug information and exit"
    echo
    echo "Warning 1: -p and -b, unlike the other flags that are sticky (because"
    echo "cmake cache them), need to be re-issued at each invocation of the"
    echo "script. Otherwise the default values will be used instead."
    echo "Warning 2: when using both -a and -l simultanously, the options"
    echo "-b and -p are ignored since they would override the platform specific"
    echo "behavior (for example -p will be the same for ArmHF and Linux)."
    echo
}

# Parse the command line arguments
parse_args()
{
    while getopts $optstring opt $@
    do
	case $opt in
	    "h") print_help
		 exit 0
		 ;;
	    "?") print_usage
		 exit 1
		 ;;
	    "l") native_build=1
		 ;;
	    "a") armhf_build=1
		 ;;
	    "p") prefix=$OPTARG
		 ;;
	    "b") build_path=$OPTARG
		 ;;
	    "B") build_type=$OPTARG
		 ;;
	    "c") clean_cache=1
		 ;;
	    "C") clean_cache=0
		 ;;
	    "d") dds_opt="-DUSE_DDS=on"
		 ;;
	    "D") dds_opt="-DUSE_DDS=off"
		 ;;
	    "r") extra_opts+=" -DENABLE_QP=on"
		 ;;
	    "R") extra_opts+=" -DENABLE_QP=off"
		 ;;
	    "f") extra_opts+=" -DENABLE_PICOFLEXX=on"
		 ;;
	    "F") extra_opts+=" -DENABLE_PICOFLEXX=off"
		 ;;
	    "k") extra_opts+=" -DUSE_CCACHE=on"
		 ;;
	    "K") extra_opts+=" -DUSE_CCACHE=off"
		 ;;
	    "v") extra_opts+=" -DENABLE_VIVE=on"
		 ;;
	    "V") extra_opts+=" -DENABLE_VIVE=off"
		 ;;
	    "t") extra_opts+=" -DENABLE_INTEGRATION_TESTING=on"
		 ;;
	    "T") extra_opts+=" -DENABLE_INTEGRATION_TESTING=off"
		 ;;
	    "g") debug_mode=1
		 ;;
	    *) print_usage
	       exit 1
	       ;;
	esac
    done
}

# Return the full canonical path of a file
# Arguments:
#   1 -> path to canonicalize
# Return:
#   0 if the path exist, 1 if the path does not exist
#   prints a string with the canonical path + store it in $canonical_path
canonicalize()
{
    freepath=$1
    os_name=`uname -s`
    case $os_name in
	Linux)
	    # just use readlink :-)
	    canonical_path=`readlink -f $freepath`
	    readl_ret=$?
	    echo $canonical_path
	    if [ $readl_ret == 1 ] ; then
		return 1
	    else
		return 0
	    fi
	    ;;
	Darwin | SunOS)
	    # BSD systems do not support readlink :-(
	    if [ -d $freepath ] ; then
		# the argument is a directory
		canonical_path=`cd $freepath && pwd -P`
	    else
		if [ -f $freepath ] ; then
		    # the argument is a file
		    freedir=`dirname $freepath`
		    freefile=`basename $freepath`
		    if [ -L $freepath ] ; then
			canfile=`cd $freedir && stat -f "%Y" $freefile`
		    else
			canfile=$freefile
		    fi
		    candir=`cd $freedir && pwd -P`
		    canonical_path="${candir}/${canfile}"
		else
		    # given path does not exsit
		    # since readlink does not return any string for this
		    # scenario, just lets do the same and return an error
		    canonical_path=""
		    return 1
		fi
	    fi
	    echo $canonical_path
	    return 0
	    ;;
	*)
	    # echo platform not supported yet
	    echo "/${os_name}/is/not/yet/supported/by/canonicalize"
	    return 1
	    ;;
    esac
}

# function to use the cmake configure with the right arguments
# arguments:
#   1: build_path
#   2: build_type
#   3: install_path
#   4: freeflyer_path
#   5: clean_cache
#   6-*: other options
configure()
{
    local build_path=$1
    shift
    local build_type=$1
    shift
    local install_path=$1
    shift
    local ff_path=$1
    shift
    local clean_cache=$1
    shift
    local cmake_opts=$@

    if [ $debug_mode == 1 ]; then
	echo "build type: ${build_type}"
	echo "build path: ${build_path}"
	echo "install directory: ${install_path}"
	echo
    else
	if [ "$install_path" != "none" ] ; then
            if [ ! -d ${install_path} ] ; then
		echo "Creating install directory: ${install_path}"
		mkdir -p $install_path
            fi
            full_install_path=`canonicalize $install_path`
            install_opt="-DCMAKE_INSTALL_PREFIX=${full_install_path}"
	fi

	if [ ! -d ${build_path} ] ; then
	    echo "Creating build directory: ${build_path}"
            mkdir -p ${build_path}
	fi
	cd ${build_path}

	if [ ${clean_cache} -eq 1 ] ; then
            echo "Remove the CMake Cache for ${build_path}"
            rm -f CMakeCache.txt
	fi
	cmd="cmake -DCMAKE_BUILD_TYPE=${build_type} ${install_opt} ${cmake_opts} ${ff_path}"
        echo $cmd # to se what we are geting
        $cmd 
    fi
}

# Start the real work here...
parse_args $@

# Define the paths to use
ff_path=`canonicalize ${rootpath}`

if [ $debug_mode == 1 ]; then
    echo "script is called from: $workdir"
    echo "script dir is: $confdir"
    echo "scriptname is: $scriptname"
    echo "absolute script path: $rootpath"
    echo "freeflyer canonical path: ${ff_path}"
    echo "DDS configuration: ${dds_opt}"
    echo "Other Options:" ${extra_opts}
    echo "linux build enabled: ${native_build}"
    echo "armhf build enabled: ${armhf_build}"
    echo
fi

if [[ $native_build == 0 && $armhf_build == 0 ]] ; then
    echo "Nothing to configure (use -l or -a)..."
    echo "Use $scriptname -h for the full list of options"
    print_usage
fi

if [[ $native_build == 1 && $armhf_build == 1 ]] ; then
    echo -n "Linux and ArmHF invoked simultanously:"
    echo " dropping any option -p and -b!"
    prefix=""
    build_path=""
fi

if [ $native_build == 1 ] ; then
    echo "configuring for native linux..."
    # Performance of the shared disk system is horrendous on Vagrant.
    # So by default we build in the home directory that is Vagrant native.
    # In addition, we do not create an install directory by default.
    # Update: we are currently forced to provide an install prefix!
    native_build_path=${build_path:-${HOME}/astrobee_build/native}
    native_install_path=${prefix:-${HOME}/astrobee_install/native}
    configure ${native_build_path} ${build_type:-RelWithDebInfo} \
	      ${native_install_path} ${ff_path} ${clean_cache} \
              ${dds_opt} ${extra_opts}
fi

if [ $armhf_build == 1 ] ; then
    echo "configuring for armhf..."
    armhf_opts="-DCMAKE_TOOLCHAIN_FILE=${ff_path}/scripts/build/ubuntu_cross.cmake -DUSE_CTC=true"
    armhf_build_path=${build_path:-${HOME}/astrobee_build/armhf}
    armhf_install_path=${prefix:-${HOME}/astrobee_install/armhf}
    configure ${armhf_build_path} ${build_type:-Release} \
	      ${armhf_install_path} ${ff_path} ${clean_cache} \
              ${dds_opt} ${armhf_opts} ${extra_opts}
fi
