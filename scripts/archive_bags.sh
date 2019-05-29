#!/bin/bash

usage()
{
  echo "Usage: archive_bags.sh [local_source_dir]"
  echo "archive bags in a folder on the local computer to the volar server."
  echo "    local_source_dir: where to a bags to archive are stored locally"
  echo "                      default=/data/bags"
  echo "    The target directory is fixed (RecordedData)"
  exit -1
}

if [[ $# -gt 0 ]]
then
  if [[ "$1" == "-h" ]] 
    then
      usage
    fi
  source_dir=$1
else
  source_dir=/data/bags
fi

if [ ! -d "$source_dir" ]
then
  usage
fi

# make sure we have a trailing slash since we want to sync the
# content under the specified directory
if [[ "$source_dir" == "${source_dir%/}" ]]
then
  source_dir="${source_dir}/"
fi

dest_host=volar.ndc.nasa.gov
dest_dir=/home/p-free-flyer/free-flyer/RecordedData

options="--archive --verbose --progress"

rsync ${options} ${source_dir} ${dest_host}:${dest_dir}
