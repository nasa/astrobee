"""
Output Python data structure as a Lua table constructor.
PLEASE NOTE: This script currently only works with python 2.
The python script that imports this script and generates the
astrobee fsw files uses a repo that is only compatible with
python 2. Until that repo is updated to work with python 3,
this script will only be compatible with python 2 since there
is no easy way to guarantee the python 3 compatibility changes
continue to generate the fsw files correctly.
"""

import sys

if sys.version_info > (3, 0):
    # exit if python 3
    print("The lua table script is only compatible with python 2")
    sys.exit(1)
from cStringIO import StringIO  # fmt: skip


def q(s):
    return '"%s"' % s


def ind(lvl):
    return " " * (lvl * 2)


def dumpStream(out, d, lvl=0):
    def w(s):
        out.write(s)

    if isinstance(d, basestring):

        w(q(d))
    # fmt: skip

    elif isinstance(d, (list, tuple)):
        if d:
            w("{\n")
            n = len(d)
            for i, elt in enumerate(d):
                w(ind(lvl + 1))
                dumpStream(out, elt, lvl + 1)
                if i < n - 1:
                    w(",")
                w("\n")
            w(ind(lvl))
            w("}")
        else:
            w("{}")

    elif isinstance(d, dict):
        if d:
            w("{\n")
            n = len(d)
            keys = list(d.keys())
            keys.sort()
            for i, k in enumerate(keys):
                v = d[k]
                w(ind(lvl + 1))
                w(k)
                w("=")
                dumpStream(out, v, lvl + 1)
                if i < n - 1:
                    w(",")
                w("\n")
            w(ind(lvl))
            w("}")
        else:
            w("{}")

    else:
        w(str(d))


def dumps(d):
    out = StringIO()
    dumpStream(out, d, 0)
    return out.getvalue()
