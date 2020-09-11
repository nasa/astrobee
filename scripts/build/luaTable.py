
"""
Output Python data structure as a Lua table constructor.
"""

from cStringIO import StringIO


def q(s):
    return '"%s"' % s


def ind(lvl):
    return ' ' * (lvl * 2)


def dumpStream(out, d, lvl=0):
    def w(s):
        out.write(s)

    if isinstance(d, basestring):
        w(q(d))

    elif isinstance(d, (list, tuple)):
        if d:
            w('{\n')
            n = len(d)
            for i, elt in enumerate(d):
                w(ind(lvl + 1))
                dumpStream(out, elt, lvl + 1)
                if i < n - 1:
                    w(',')
                w('\n')
            w(ind(lvl))
            w('}')
        else:
            w('{}')

    elif isinstance(d, dict):
        if d:
            w('{\n')
            n = len(d)
            keys = d.keys()
            keys.sort()
            for i, k in enumerate(keys):
                v = d[k]
                w(ind(lvl + 1))
                w(k)
                w('=')
                dumpStream(out, v, lvl + 1)
                if i < n - 1:
                    w(',')
                w('\n')
            w(ind(lvl))
            w('}')
        else:
            w('{}')

    else:
        w(str(d))


def dumps(d):
    out = StringIO()
    dumpStream(out, d, 0)
    return out.getvalue()
