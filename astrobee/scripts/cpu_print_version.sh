#!/bin/bash

function deb_version {
  VERSION=$(dpkg -s $1 2>/dev/null | grep -oP 'Version: \K.+')
  if [ $? -eq 0 ]; then
    echo $VERSION
  else
    echo "None Installed"
  fi
}

source /opt/ros/kinetic/setup.bash
date
echo "Kernel: $(uname -r)"
lsb_release -r
echo "Ros $(rosversion -d) $(rosversion roscpp)"
echo "Debians:"
echo "  astrobee-comms:     $(deb_version astrobee-comms)"
echo "  astrobee0:          $(deb_version astrobee0)"
echo "  astrobee-config:    $(deb_version astrobee-config)"
echo "  astrobee-avionics:  $(deb_version astrobee-avionics)"
echo "FSW Version:"
cat /opt/astrobee/version.txt | sed 's/^/  /'
echo "Modified Files:"
dpkg -V astrobee-comms | sed 's/^/  /'
dpkg -V astrobee0 | sed 's/^/  /'
dpkg -V astrobee-config | sed 's/^/  /'
dpkg -V astrobee-avionics | sed 's/^/  /'
