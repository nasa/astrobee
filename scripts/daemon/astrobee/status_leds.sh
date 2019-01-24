#!/bin/bash
#
# Control the status LEDs from various system checks 
#

export PATH="/opt/astrobee/bin:/opt/ros/kinetic/bin:$PATH"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"

# Arrays of LED indexes, names and states
keys=("llp" "mlp" "hlp" "net" "ros" "fsw" "video" "audio" "streaming")
leds=("c1" "b1" "a1" "c2" "b2" "a2" "video" "audio" "live")
states=("na" "na" "na" "na" "na" "na" "na" "na" "na")

# Print message is DBG is 1
DBG=0

#
# Retrieve the index of the array from the key
# Argument:
#   1 -> key to search for
# Return:
#   0..n -> index corresponding to the key
#   255 -> key not found
#
getKeyIndex()
{
  for i in `seq 0 $((${#keys[@]}-1))` ; do
	   if [ ${keys[$i]} == $1 ] ; then
	      # echo "key for [$1] = $i"
        return $i
	     fi
  done
  # echo "key not found!"
  return 255
}

#
# Print a timestamped message on the console
#
msg () {
  if [ $DBG -gt 0 ] ; then
    now=`date "+%Y-%m-%d_%H:%M:%S"`
    printf "%s | %b\\n" $now "$*" >&2
  fi
}


# Set a LED on or off using either a service call or the driver tool
# Arguments:
#   - 1 : led_name (string)
#   - 2 : state (number mapping in eps_driver_tool.cc)
# This function abstract the LED configuration to avoid potential
# contentious I2C device access by the command line tool versus service
set_led ()
{
  getKeyIndex $1
  index=$?
  state_only=${3:-false}
  if [ $index -ge 0 ] ; then
    if [ "${states[$index]}" != $2 ] ; then
      states[$index]="$2"
      # echo "command led=${leds[$index]} set=${states[$index]}"
      [ "$state_only" = "false" ] && \
        eps_driver_tool -led "${leds[$index]}" -set "${states[$index]}"  &> /dev/null
    fi
  fi
}

# When LLP is up (this script is call started), warn that NET and FSW are not
# ready
msg "startup with all amber ON (unkown state)"
set_led net on
set_led fsw on
set_led ros on

msg "declare video is on by default"
set_led video on

# Since this script is started, LLP is up ;-)
set_led llp on

# Another service is turning ON MLP and HLP: it will start blinking the 
# corresponding green LEDs
mlp_up=0
hlp_up=0

msg "looping for ever..."
# Repeat while we are up...
while :
do
  # Check if MLP is up
  if ping -c1 mlp &> /dev/null ; then
    msg "MLP is up"
    set_led mlp on
    mlp_up=1
  else
    msg "MLP is down"
    if [ $mlp_up -eq 1 ] ; then
      # MLP went down!
      set_led mlp error
    else
      set_led mlp off
    fi
  fi
  sleep 1

  # Check if HLP is up
  if ping -c1 hlp  &> /dev/null ; then
    msg "HLP is up"
    set_led hlp on
    hlp_up=1
  else
    msg "HLP is down"
    if [ $hlp_up -eq 1 ] ; then
      # HLP went down!
      set_led hlp error
    else
      set_led hlp off
    fi
  fi
  sleep 1

  # Check if connectivity to dock OK
  if ssh -o 'PreferredAuthentications=publickey' mlp ping -c1 dock-iss &> /dev/null ; then
    msg "Dock ping OK"
    set_led net off
  else
    msg "Dock ping failed"
    set_led net on
  fi
  sleep 1

  # Check if roscore is runing
  #if rostopic list &> /dev/null ; then
  r=`systemctl --user is-active ros`
  if [ "$r" == "active" ] ; then
    msg "roscore up"
    set_led ros off
  else
    msg "no roscore"
    set_led ros on
  fi
  sleep 1

  # Check if something like a FSW stack is running
  launched=`ps -aef |grep roslaunch | wc -l`
  if [ "$launched" -lt 2 ]; then
    msg "FSW down"
    set_led fsw on
  else
    msg "FSW up, assuming LED is off"
    set_led fsw off true
  fi
  sleep 1

done

