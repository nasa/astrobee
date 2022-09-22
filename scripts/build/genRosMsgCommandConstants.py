#!/usr/bin/env python
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
A library and command-line tool for generating a ROS msg file containing
declarations of command names, from the Astrobee XPJSON schema.
"""

import os
import sys
import re
import logging

# hack to set up PYTHONPATH
ffroot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(ffroot, 'astrobee', 'commands', 'xgds_planner2'))

from xgds_planner2 import xpjson
import xpjsonAstrobee


TEMPLATE_MAIN = '''
# Copyright (c) 2015 United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.

%(decls)s
'''[1:]
# END TEMPLATE_MAIN

TEMPLATE_DECL = '''
string CMD_NAME_%(baseIdAllCaps)s = %(baseId)s
'''[1:-1]

TEMPLATE_PARAM_DECL = '''
string PARAM_NAME_%(baseIdAllCaps)s_%(choiceLabelAllCaps)s = %(choiceCode)s
'''[1:-1]

TEMPLATE_SUBSYS_DECL = '''
string CMD_SUBSYS_%(categoryAllCaps)s = %(category)s
'''[1:-1]

def splitCommandCategory(cmd):
    assert '.' in cmd.id, 'CommandSpec without category: %s' % cmd
    category, baseId = cmd.id.split('.', 1)
    return category, baseId

def getCommandContext(cmd):
    category, baseId = splitCommandCategory(cmd)
    return {
        'categoryAllCaps': xpjsonAstrobee.allCaps(category),
        'baseId': baseId,
        'baseIdAllCaps': xpjsonAstrobee.allCaps(baseId),
        'fullId': cmd.id,
    }

def getParamContext(param, code, label):
    assert '.' in param.id, 'ParamSpec without category: %s' % param
    category, baseId = param.id.split('.', 1)
    return {
        'baseIdAllCaps': xpjsonAstrobee.allCaps(baseId),
        'choiceLabelAllCaps': xpjsonAstrobee.allCaps(label),
        'choiceCode': code,
    }

def genCommandNamesMsg(inSchemaPath, outCommandNamesPath):
    schema = xpjsonAstrobee.loadDocument(inSchemaPath)

    paramSpecs = sorted(schema.paramSpecs, key=lambda c: c.id)
    cmdSpecs = sorted(schema.commandSpecs, key=lambda c: c.id)

    seenCategories = {}
    declList = []

    for paramSpec in paramSpecs: 
        if not paramSpec.choices:
            continue
        for choiceCode, choiceLabel in paramSpec.choices:
            declList.append(TEMPLATE_PARAM_DECL % getParamContext(paramSpec, choiceCode, choiceLabel))

    declList.append('')

    for cmdSpec in cmdSpecs:
        category, _ = splitCommandCategory(cmdSpec)
        seenCategories[category] = True
        declList.append(TEMPLATE_DECL % getCommandContext(cmdSpec))

    declList.append('')

    for key in sorted(seenCategories.keys()):
        declList.append(TEMPLATE_SUBSYS_DECL % {
            'categoryAllCaps': xpjsonAstrobee.allCaps(key),
            'category': key,
        })

    decls = '\n'.join(declList)

    with open(outCommandNamesPath, 'w') as outStream:
        outStream.write(TEMPLATE_MAIN % {'decls': decls})
    logging.info('wrote ROS msg file declaring command name constants to %s', outCommandNamesPath)


def main():
    import optparse
    parser = optparse.OptionParser('usage: %prog <inSchema.json> [CommandConstants.msg]\n\n' + __doc__.strip())
    opts, args = parser.parse_args()
    if len(args) == 2:
        inSchemaPath, outCommandNamesPath = args
    elif len(args) == 1:
        inSchemaPath = args[0]
        outCommandNamesPath = 'CommandConstants.msg'
    else:
        parser.error('expected 1 or 2 args')
    logging.basicConfig(level=logging.DEBUG, format='%(message)s')
    genCommandNamesMsg(inSchemaPath, outCommandNamesPath)


if __name__ == '__main__':
    main()
