#!/bin/bash

# Usage: clock_sync_monitor.sh [ --once ]
#
# Periodically log simultaneous timestamps from multiple processors so
# as to watch for clock skew (and possibly correct it in
# post-processing).
#
# With no arguments, the script daemonizes and logs clock sync
# periodically in an infinite loop.  If --once is specified, it will not
# daemonize and just logs clock sync once.

processUSR1() {
    exit 0
}

trap processUSR1 SIGUSR1

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
THIS_FILE=$(basename $0)
cd /

# Optional sanity check to avoid running multiple daemons simultaneously. Can comment this out if pgrep isn't installed.
num_processes=$(pgrep -fc "$THIS_DIR/$THIS_FILE")
if [ "$num_processes" -gt 1 ]; then
    >&2 echo "There appears to be another instance of this process running. Try: killall $THIS_FILE"
    exit 1
fi

# PARENT - Start daemon setup and fork to child1
if [ "$1" == "" ]; then
    setsid $THIS_DIR/$THIS_FILE child1 &
    exit 0
fi

# CHILD 1 - Do more daemon setup and fork to child2
if [ "$1" == "child1" ]; then
    shift

    umask 0
    $THIS_DIR/$THIS_FILE child2 </dev/null >/dev/null 2>/dev/null &
    exit 0
fi

# CHILD 2 - Now we are running as a daemon, do the real work

# Redirect stdout and stderr to log file. No stdin input.
# TODO: Write log file within a folder that will get downlinked.
exec >> /tmp/clock_sync_monitor.log
exec 2>&1
exec 0< /dev/null

log_remote_clock_sync() {
    remote_host="$1"
    # Output timestamps in ISO 8601 format (both human- and machine-readable).
    # Note: date %N nanoseconds format code is not supported on all platforms, but seems to be ok in recent Ubuntu.
    # If the remote host is available, we will log a 0 exit value and the remote time.
    # If the remote host is not available, we will log a non-zero exit value and the error message.
    remote_time_or_error=$((ssh -x -o ConnectTimeout=1 "$remote_host" 'date -u "+%Y-%m-%dT%H:%M:%S.%NZ"') 2>&1)
    ssh_exit_value="$?"
    local_time=$(date -u "+%Y-%m-%dT%H:%M:%S.%NZ")
    echo "$local_time" clock "$remote_host" "$ssh_exit_value" "$remote_time_or_error"
}

# >&2 echo "bogus error"

while true; do
    # This should fail gracefully if some hosts are not available. The
    # other hosts will still be logged properly.  We could make
    # variants of this script that append payload processors to this
    # list if they expose sshd (e.g. SoundSee).
    log_remote_clock_sync mlp
    log_remote_clock_sync hlp

    if [ "$1" == "--once" ]; then
	break
    fi

    # 10 minutes
    sleep 600
done
