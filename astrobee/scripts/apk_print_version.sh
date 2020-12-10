#!/bin/bash

function apk_version {
  VERSION=$(adb shell dumpsys package $1 | grep versionName)
  if [ $? -eq 0 ]; then
    echo $VERSION
  else
    echo "None Installed"
  fi
}

adb connect hlp

echo "APKs:"
echo "  cpu monitor:             $(apk_version gov.nasa.arc.astrobee.cpu_monitor)"
echo "  disk monitor:            $(apk_version gov.nasa.arc.astrobee.disk_monitor)"
echo "  guest science manager:   $(apk_version gov.nasa.arc.astrobee.android.gs.manager)"
echo "  mic test:                $(apk_version gov.nasa.arc.irg.astrobee.mictest)"
echo "  signal intention state:  $(apk_version gov.nasa.arc.astrobee.signal_intention_state)"
echo "  sci cam:                 $(apk_version gov.nasa.arc.astrobee.set_wallpaper)"
echo "  set wallpaper:           $(apk_version gov.nasa.arc.irg.astrobee.wifisetup)"
echo "  wifi setup helper:       $(apk_version gov.nasa.arc.irg.astrobee.battery_monitor)"
