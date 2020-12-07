#!/bin/bash

if [ `hostname` != mlp ] ; then
    echo "Must be run on mlp."
    exit 1
fi

echo "=================== MLP ==================="
./cpu_print_version.sh | sed 's/^/  /'

echo "=================== LLP ==================="
ssh llp /opt/astrobee/lib/astrobee/cpu_print_version.sh | sed 's/^/  /'

echo "=================== HLP ==================="

./apk_print_version.sh | sed 's/versionName=/  /'
  
echo "=================== Map ==================="
echo "Map: $(sha256sum /res/maps/iss.map)"
