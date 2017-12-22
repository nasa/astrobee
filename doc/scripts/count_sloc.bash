#!/bin/bash

# need cloc to work...

confdir=`dirname $0`
rootpath=`cd $confdir/../..; pwd`

dirs_to_exclude=doc/html,gnc/matlab,submodules/avionics,scripts/git,android/picoflexx
lang_defs=$rootpath/doc/scripts/languages_definition.txt

cloc --force-lang-def=$lang_defs --exclude-dir=$dirs_to_exclude $rootpath

