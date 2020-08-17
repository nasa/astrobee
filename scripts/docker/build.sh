#!/bin/bash

# short help
usage_string="$scriptname [-h] [-a <astrobee source path>] [-i <isaac source path>] [-d <idi source path>]"
#[-t make_target]

usage()
{
    echo "usage: sysinfo_page [[[-a file ] [-i]] | [-h]]"
}
ubuntu18=0

while [ "$1" != "" ]; do
    case $1 in
        -a | --astrobee_source_dir )   shift
                                		astrobee_source=$1
                                		;;
        -n | --ubuntu18 )               ubuntu18=1
                                        ;;
        -h | --help )           		usage
                                		exit
                                		;;
        * )                     		usage
                                		exit 1
    esac
    shift
done


thisdir=$(dirname "$(readlink -f "$0")")
rootdir=${thisdir}/../../..
echo "Astrobee path: "${astrobee_source:-${rootdir}/astrobee/}
if [ $ubuntu18 == 0 ]; then
    docker build --no-cache ${astrobee_source:-${rootdir}/astrobee/} -f ${astrobee_source:-${rootdir}/astrobee/}scripts/docker/Dockerfile_astrobee -t astrobee
else
    docker build --no-cache ${astrobee_source:-${rootdir}/astrobee/} -f ${astrobee_source:-${rootdir}/astrobee/}scripts/docker/Dockerfile_astrobee18 -t astrobee
fi

