#!/bin/bash -e
USERNAME="${NDC_USERNAME:-$USER}"
URI=$USERNAME@volar.ndc.nasa.gov:/home/p-free-flyer/free-flyer/SimulationMedia
FILE=astrobee_media-$1.tar.gz
if [ ! -f "$FILE" ]; then
    echo "Media assets version $1 not found. Downloading..."
    scp $URI/$FILE $FILE
else
    echo "Media assets version $1 found. Skipping download."
fi
echo "Extracting media assets from ${FILE}."
tar -xzf $FILE
