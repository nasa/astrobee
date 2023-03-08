#!/bin/bash

# Updates the src_paths setting in the .isort.cfg file at the top
# level. See that file for more details.

thisdir=$(dirname "$0")
srcdir=$(cd $thisdir/../.. && pwd)

cd $srcdir

# Generate a comma-separated list of folders containing *.py files
pydirs=$(find . -name "*.py" -print0 | xargs -0 dirname | cut -c3- | sort | uniq | paste -sd "," -)

# Overwrite the src_paths line in the config file to use the list
perl -i -ple "if (/^src_paths = /) { \$_ = 'src_paths = $pydirs'; }" .isort.cfg
