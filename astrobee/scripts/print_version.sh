#!/bin/bash

# Print the software versions on the MLP, LLP, and HLP.

if [ `hostname` != mlp ] ; then
    echo "Must be run on mlp."
    exit 1
fi

DIR=$(dirname "$(readlink -f "$0")")

echo "=================== MLP ==================="
$DIR/cpu_print_version.sh | sed 's/^/  /'

echo "=================== LLP ==================="
ssh llp /opt/astrobee/lib/astrobee/cpu_print_version.sh | sed 's/^/  /'

echo "=================== HLP ==================="

$DIR/apk_print_version.sh | sed 's/versionName=/  /'
  
echo "=================== Map ==================="
echo "Map: $(sha256sum /res/maps/iss.map)"
