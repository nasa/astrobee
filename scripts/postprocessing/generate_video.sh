#!/bin/bash

#
# Generate videos for each set of imagery under the current diretory
#

for d in `find . -type d -maxdepth 2 -mindepth 2`
do
    cam=${d##*/}
    dir=${d%/*}
    echo "processing $dir for $cam"
    first=`ls -1 $d | head -n 1`
    ffmpeg -r 5 -start_number ${first%.jpg} -i ${d}/%04d.jpg ${dir}_${cam}.mp4
done


