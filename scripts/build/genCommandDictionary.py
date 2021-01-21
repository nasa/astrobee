#!/usr/bin/env python

"""
A command-line tool for generating an HTML command dictionary from
an XPJSON schema.
"""

import os
import sys
import logging

# hack to ensure xgds_planner2 submodule is at head of PYTHONPATH
ffroot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ffroot, 'astrobee', 'commands', 'xgds_planner2'))

from xgds_planner2 import commandDictionary

def main():
    import optparse
    parser = optparse.OptionParser('usage: %prog <inSchemaPath> <outHtmlPath>\n\n' + __doc__.strip())
    opts, args = parser.parse_args()
    if len(args) == 2:
        inSchemaPath, outHtmlPath = args
    else:
        parser.error('expected exactly 2 args')
    logging.basicConfig(level=logging.DEBUG, format='%(message)s')
    commandDictionary.writeCommandDictionary(inSchemaPath, outHtmlPath,
                                             includeCommandSpecNameField=False,
                                             includeCommandSpecNotesField=False)


if __name__ == '__main__':
    main()
