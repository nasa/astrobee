#!/bin/bash

if [[ $# -lt 1 ]] 
then
  echo "Usage: pull_data.sh source_host [local_backup_dir]"
  echo "rsync bags from a robot to a local directory"
  echo "    source_host:      host to retrieve the bags from. Everything from"
  echo "                      /data/bags is synched on the local computer"
  echo "    local_backup_dir: where to archive locally the retrieved bags"
  echo "                      default=/data/bags"
  exit -1
fi

if [[ "$1" == "localhost" ]]
then
  source_host=""
else
  source_host="$1:"
fi

source_dir=/data/bags/
# trailing slash is critical: we want all the content of the directory...

if [[ $# -gt 1 ]]
then
  target_dir=$2
else
  target_dir=/data/bags
fi

options="--archive --verbose --progress"

rsync ${options} ${source_host}${source_dir} ${target_dir}
