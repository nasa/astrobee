#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import sys
import argparse

try:
    from catkin_pkg.package import parse_package
except ImportError as e:
    sys.exit('ImportError: "from catkin_pkg.package import parse_package" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)


def _get_output(package):
    """
    returns a list of strings with cmake commands to execute to set cmake variables

    :param package: Package object
    :returns: list of str, lines to output
    """
    values = {}
    values['VERSION'] = '"%s"' % package.version

    values['MAINTAINER'] = '"%s"' % (', '.join([str(m) for m in package.maintainers]))

    values.update(_get_dependency_values('BUILD_DEPENDS', package.build_depends))
    values.update(_get_dependency_values('BUILDTOOL_DEPENDS', package.buildtool_depends))
    values.update(_get_dependency_values('RUN_DEPENDS', package.run_depends))

    deprecated = [e.content for e in package.exports if e.tagname == 'deprecated']
    values['DEPRECATED'] = '"%s"' % ((deprecated[0] if deprecated[0] else 'TRUE') if deprecated else '')

    output = []
    output.append(r'set(_CATKIN_CURRENT_PACKAGE "%s")' % package.name)
    for k, v in values.items():
        output.append('set(%s_%s %s)' % (package.name, k, v))
    return output

def _get_dependency_values(key, depends):
    values = {}
    values[key] = ' '.join(['"%s"' % str(d) for d in depends])
    for d in depends:
        comparisons = ['version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']
        for comp in comparisons:
            value = getattr(d, comp, None)
            if value is not None:
                values['%s_%s_%s' % (key, str(d), comp.upper())] = '"%s"' % value
    return values


def main(argv=sys.argv[1:]):
    """
    Reads given package_xml and writes extracted variables to outfile.
    """
    parser = argparse.ArgumentParser(description="Read package.xml and write extracted variables to stdout")
    parser.add_argument('package_xml')
    parser.add_argument('outfile')
    args = parser.parse_args(argv)
    package = parse_package(args.package_xml)

    lines = _get_output(package)
    with open(args.outfile, 'w') as ofile:
        ofile.write('\n'.join(lines))


if __name__ == '__main__':
    main()
