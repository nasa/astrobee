#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
APTLY="$DIR/aptly/aptly -config=$DIR/aptly.conf"

parse_packages() {
  local filename="$*"
  var=`fgrep "packages=" $filename | cut -f 2 -d "="`
  # remove leading whitespace characters
  var="${var#"${var%%[![:space:]]*}"}"
  # remove trailing whitespace characters
  var="${var%"${var##*[![:space:]]}"}"   
  packages=`echo -n "$var"`
  packages=${packages//[[:space:]]/ | }
  echo -n $packages
  #echo -n ${packages// / | }
}

contains_element () {
  local e match="$1"
  shift
  for e; do [[ "$e" == "$match" ]] && return 1; done
  return 0
}

DEBLOC=${ASTROBEE_DEBIAN_DIR:-/home/p-free-flyer/free-flyer/FSW/ars_debs/dists/xenial/main}

# add our debians
$APTLY repo add astrobee $DEBLOC/binary-armhf/*.deb
$APTLY repo add astrobee $DEBLOC/binary-amd64/*.deb
$APTLY repo add astrobee $DEBLOC/source/*.dsc

# needs latest conf files from repo
packages=$(parse_packages $DIR/astrobee_conf/*.conf)
IFS=' | ' read -r -a array <<< "$packages"
packages=("${array[@]}")
  
package_index="0"

# try multiple times to get dependencies recursively
for i in `seq 1 10`;
do
  # first update with current list of packages
  filter="${packages[@]}"
  filter=${filter// / | }

  # update mirror fitlers
  $APTLY mirror edit -architectures=armhf -filter="Priority (required) | $filter" -filter-with-deps -dep-follow-recommends -dep-follow-all-variants xenial-main
  $APTLY mirror edit -architectures=armhf -filter="Priority (required) | $filter" -filter-with-deps -dep-follow-recommends -dep-follow-all-variants xenial-updates
  $APTLY mirror edit -architectures=armhf -filter="Priority (required) | $filter" -filter-with-deps -dep-follow-recommends -dep-follow-all-variants xenial-security
  $APTLY mirror edit -architectures=armhf -filter="Priority (required) | $filter" -filter-with-deps -dep-follow-recommends -dep-follow-all-variants ros

  # update mirrors and create array of unsatisfied dependencies
  readarray d0 <<< `$APTLY mirror update -dep-verbose-resolve xenial-main | grep "Unsatisfied dependency: " | sed -n -e 's/\[armhf\]//g' -e 's/([^()]*)//g' -e 's/Unsatisfied dependency: //gp' `
  readarray d1 <<< `$APTLY mirror update -dep-verbose-resolve xenial-updates | grep "Unsatisfied dependency: " | sed -n -e 's/\[armhf\]//g' -e 's/([^()]*)//g' -e 's/Unsatisfied dependency: //gp' `
  readarray d2 <<< `$APTLY mirror update -dep-verbose-resolve xenial-security | grep "Unsatisfied dependency: " | sed -n -e 's/\[armhf\]//g' -e 's/([^()]*)//g' -e 's/Unsatisfied dependency: //gp' `
  readarray d3 <<< `$APTLY mirror update -dep-verbose-resolve ros | grep "Unsatisfied dependency: " | sed -n -e 's/\[armhf\]//g' -e 's/([^()]*)//g' -e 's/Unsatisfied dependency: //gp' `
  dependencies=("${d0[@]}" "${d1[@]}" "${d2[@]}" "${d3[@]}")

  # find dependencies that are new (some may just be in a different mirror so marked unsatisfied)
  newdeps=()
  for R in ${dependencies[@]} ;
  do
    if contains_element "$R" "${packages[@]}" ; then
      newdeps+=("$R")
    fi
  done

  # go through all packages added since last iteration
  while [ $package_index -lt ${#packages[@]} ]
  do
      echo "$package_index ${packages[package_index]}"
      # search for dependencies in all mirrors
      read -r -a d0 <<< `$APTLY mirror search -format="{{.Package}}" -with-deps xenial-main ${packages[package_index]} 2> /dev/null`
      read -r -a d1 <<< `$APTLY mirror search -format="{{.Package}}" -with-deps xenial-updates ${packages[package_index]} 2> /dev/null`
      read -r -a d2 <<< `$APTLY mirror search -format="{{.Package}}" -with-deps xenial-security ${packages[package_index]} 2> /dev/null`
      read -r -a d3 <<< `$APTLY mirror search -format="{{.Package}}" -with-deps ros ${packages[package_index]} 2> /dev/null`
      read -r -a d4 <<< `$APTLY mirror search -format="{{.Package}}" -with-deps astrobee ${packages[package_index]} 2> /dev/null`
      dependencies=( "${d0[@]}" "${d1[@]}" "${d2[@]}" "${d3[@]}" "${d4[@]}" )
      # keep ones that are new
      for R in ${dependencies[@]} ;
      do
        if contains_element "$R" "${packages[@]}" ; then
          newdeps+=("$R")
        fi
      done
      package_index=$[$package_index+1]
  done

  echo "New dependencies: ${newdeps[@]}"
  if [ ${#newdeps[@]} -eq 0 ]; then
    break
  fi
  # remove duplicates from newdeps and add to packages list
  temp1=($(echo "${newdeps[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))
  temp2=("${packages[@]}" "${temp1[@]}")
  packages=("${temp2[@]}")
  echo "Mirroring ${#packages[@]} packages: ${packages[@]}."
done
