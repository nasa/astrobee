#!/bin/bash

if [ $# -eq 0 ]
  then
    echo "Specify snapshot name."
    exit 0
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
APTLY="$DIR/aptly/aptly -config=$DIR/aptly.conf"

./create_version_debian.sh $1
$APTLY repo add astrobee astrobee-repo-version*.deb
rm astrobee-repo-version*.deb

$APTLY snapshot create astrobee_$1 from repo astrobee || exit
$APTLY snapshot create main_$1 from mirror xenial-main || exit
$APTLY snapshot create security_$1 from mirror xenial-security || exit
$APTLY snapshot create updates_$1 from mirror xenial-updates || exit
$APTLY snapshot create ros_$1 from mirror ros || exit

$APTLY snapshot merge -latest $1 astrobee_$1 main_$1 security_$1 updates_$1 ros_$1 || exit

$APTLY snapshot create bionic-astrobee_$1 from repo bionic-astrobee || exit

# need to delete old version first
$APTLY publish drop xenial
$APTLY publish drop bionic

$APTLY publish snapshot -gpg-key=33C0A17A -distribution="xenial" $1 || exit 0
$APTLY publish snapshot -gpg-key=33C0A17A -distribution="bionic" bionic-astrobee_$1 || exit 1

PUBLISH_DIR=`$APTLY config show | grep "rootDir" | sed -r 's|^[^:]+: "([^"]+)",$|\1|'`/public

chmod g+w -R --silent $PUBLISH_DIR

kinit || exit 1
rsync -ah --no-t --delete $PUBLISH_DIR/dists/ astrobee.ndc.nasa.gov:/home/irg/software/dists
rsync -ah --no-t --delete $PUBLISH_DIR/pool/ astrobee.ndc.nasa.gov:/home/irg/software/pool

