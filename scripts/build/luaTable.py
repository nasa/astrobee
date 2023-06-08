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
Output Python data structure as a Lua table constructor.
"""

import sys
from typing import Any, Dict, List, Tuple, Union

try:
    from cStringIO import StringIO  # fmt: skip
except ImportError:
    # Python 3
    from io import StringIO

if sys.version_info.major > 2:
    basestring = (str, bytes)


def q(s: str) -> str:
    return f'"{s}"'


def ind(lvlL: int) -> str:
    return " " * (lvl * 2)


def dumpStream(out: StringIO, d: Any, lvl: int = 0) -> None:
    lines = []
    def w(s):
        out.append(s)

    if isinstance(d, basestring):
        w(q(d))
    # fmt: skip

    elif isinstance(d, (list, tuple)):
        if d:
            w("{")
            for elt in d:
                w(ind(lvl + 1))
                dumpStream(out, elt, lvl + 1)
                w(",")
            w(ind(lvl) + "}")
        else:
            w("{}")

    elif isinstance(d, dict):
        if d:
            w("{\n")
            keys = sorted(d.keys())
            for k in keys:
                v = d[k]
                w(ind(lvl + 1))
                w(k)
                w("=")
                dumpStream(out, v, lvl + 1)
        else:
            w("{}")
    else:
        w(str(d))
    out.write("\n".join(lines))


def dumps(d: Any) -> str:
    out = StringIO()
    dumpStream(out, d, 0)
    return out.getvalue()
