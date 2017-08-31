#!/usr/bin/env python

from __future__ import print_function

from argparse import ArgumentParser
import os
import sys

try:
    from catkin_pkg.workspaces import order_paths
except ImportError as e:
    sys.exit('ImportError: "from catkin_pkg.package import parse_package" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)


def main():
    """
    Order a list of paths according to a list of prefixes which define the order.
    """
    parser = ArgumentParser(description='Utility to order a list of paths according to a list of prefixes. Creates a file with CMake set command setting a variable.')
    parser.add_argument('outfile', help='The filename of the generated CMake file')
    parser.add_argument('--paths-to-order', nargs='*', help='The semicolon-separated paths to order')
    parser.add_argument('--prefixes', nargs='*', help='The semicolon-separated prefixes defining the order')
    args = parser.parse_args()

    ordered_paths = order_paths(args.paths_to_order, args.prefixes)

    # create directory if necessary
    outdir = os.path.dirname(args.outfile)
    if not os.path.exists(outdir):
        os.makedirs(outdir)

    with open(args.outfile, 'w') as fh:
        fh.write('set(ORDERED_PATHS "%s")' % ';'.join(ordered_paths))


if __name__ == '__main__':
    main()
