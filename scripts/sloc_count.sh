#
# Count SLOC in the Astrobee Robot Software source tree
#
# The following is ignored:
#   - submodule/avionics -- Firmware developed partly by the Avionics Team
#   - Simulink models and generated code -- GNC Team mostly
#   - Few contributed code that is in the repo for convinience
#   - All generated code (C++ and Java)
#
# Relevant language categories:
#   - C++ and Header : Obviously most our code base (includes few C files)
#   - Java : Support for guest science (Android)
#   - Lua and HTML : System Configuration
#     - Lua is all the FSW .config files
#     - HTML (wrong name) groups URDF descriptions and custom ROS Launch files
#   - IDL : Message definition (includes ROS messages, service and actions,
#           but comments count is wrong)
#   - Bourne Again Shell : Mostly platform management and deployment
#   - Python : Collection of utilities
#
# Not relevant languages:
#   - XML : counts too many project configuration in addition to our own code
#           now just filtered out of the count, waiting for better option
#

callcmd=$0
workdir=`pwd`
scriptdir=`dirname $callcmd`
scriptname=${0##*/}
rootpath=`cd $scriptdir/..; pwd`

fsw_dirs="astrobee behaviors cmake communications description hardware"
fsw_dirs+=" localization management mobility scripts shared simulation tools wdock"

dirs_to_exclude="code_generation,hardware/embedded,scripts/git"
dirs_to_exclude+=",submodules/android/astrobee_api/api/src/main/generated"
dirs_to_exclude+=",submodules/avionics"
dirs_to_exclude+=",xgds_planner2"
dirs_to_exclude+=",busybox_patches,kernel_patches,uboot_paches,inforce/patches"

include_submodules="submodules"
#include_submodules=""

cd $scriptdir/..
cloc --exclude-dir=$dirs_to_exclude \
  --exclude-list-file=$rootpath/scripts/files_to_exclude_from_count.txt \
  --force-lang="Lua",config --force-lang="Bourne Again Shell",sh \
  --force-lang="IDL",msg --force-lang="IDL",srv --force-lang="IDL",action \
  --force-lang="Bourne Again Shell",service --force-lang="C++",c \
  --force-lang="HTML",xacro --force-lang="HTML",urdf --force-lang="HTML",launch \
  --exclude-ext=bat,yaml,xml \
  $fsw_dirs $include_submodules
cd -
