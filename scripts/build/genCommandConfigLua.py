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
A library and command-line tool for generating a RAPID-style CommandConstants.idl
file from an XPJSON schema.
"""

import argparse
import logging
import os
import re
import sys

# hack to set up PYTHONPATH
ffroot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(ffroot, "astrobee", "commands", "xgds_planner2"))

import luaTable
import xpjsonAstrobee
from xgds_planner2 import xpjson

TEMPLATE_MAIN = """
-- Copyright (c) 2015 United States Government as represented by the
-- Administrator of the National Aeronautics and Space Administration.
-- All Rights Reserved.

commandConfig = %(table)s
"""
# END TEMPLATE_MAIN


def getParamConfig(param):
    if "." in param.id:
        category, baseId = param.id.split(".", 1)
    else:
        category, baseId = None, param.id

    return {
        "key": xpjsonAstrobee.fixName(baseId),
        "type": xpjsonAstrobee.XPJSON_PARAM_VALUE_TYPE_MAPPINGS[param.valueType],
    }


def getCommandConfig(cmd):
    assert "." in cmd.id, "CommandSpec without category: %s" % cmd
    category, baseId = cmd.id.split(".", 1)
    return {
        "name": baseId,
        "parameters": [getParamConfig(p) for p in cmd.params],
    }


def genCommandConfigLua(inSchemaPath, outCommandConfigPath):
    schema = xpjsonAstrobee.loadDocument(inSchemaPath)
    specs = sorted(schema.commandSpecs, key=lambda c: c.id)

    categoryMap = {}
    for spec in specs:
        assert "." in spec.id, "CommandSpec without category: %s" % spec
        category, baseId = spec.id.split(".", 1)
        specsInCategory = categoryMap.setdefault(category, [])
        specsInCategory.append(getCommandConfig(spec))

    categories = sorted(categoryMap.keys())
    config = {
        "availableSubsystems": [
            {"name": c, "subsystemTypeName": c + "Type"} for c in categories
        ],
        "availableSubsystemTypes": [
            {"name": k + "Type", "commands": categoryMap[k]} for k in categories
        ],
    }
    # import json; print json.dumps(config, indent=4, sort_keys=True)
    table = luaTable.dumps(config)

    with open(outCommandConfigPath, "w") as outStream:
        outStream.write(TEMPLATE_MAIN % {"table": table})
    logging.info("wrote command config Lua to %s", outCommandConfigPath)


class CustomFormatter(
    argparse.RawDescriptionHelpFormatter,
    argparse.ArgumentDefaultsHelpFormatter
):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__ + "\n\n",
        formatter_class=CustomFormatter,
    )
    parser.add_argument(
        "inSchemaPath",
        help="input XPJSON schema path",
    )
    parser.add_argument(
        "outCommandConfigPath",
        help="output Lua command config file",
        nargs="?",
        default="commands.config",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG, format="%(message)s")
    genCommandConfigLua(args.inSchemaPath, args.outCommandConfigPath)


if __name__ == "__main__":
    main()
