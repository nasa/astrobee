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
echo "  astrobee:           $(deb_version astrobee)"
echo "  astrobee-config:    $(deb_version astrobee-config)"
echo "  astrobee-avionics:  $(deb_version astrobee-avionics)"
echo "FSW Version:"
cat /opt/astrobee/version.txt | sed 's/^/  /'
echo "Modified Files:"
dpkg -V astrobee | sed 's/^/  /'
dpkg -V astrobee-config | sed 's/^/  /'
dpkg -V astrobee-avionics | sed 's/^/  /'
#echo "  astrobee: $(dpkg -s astrobee | grep -oP 'Version: \K.+')"
#echo "  astrobee-config: $(dpkg -s astrobee-config | grep -oP 'Version: \K.+')"
#echo "  astrobee-avionics: $(dpkg -s astrobee-avionics | grep -oP 'Version: \K.+')"
#echo ""
#echo "Version:"
#cat /opt/astrobee/version.txt
