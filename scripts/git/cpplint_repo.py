#!/usr/bin/env python

import os, imp, fnmatch, subprocess
from operator import itemgetter
from itertools import groupby

num_errors = 0

def get_cpplint_path():
    return os.path.dirname(os.path.realpath(__file__)) + "/cpplint.py"

def get_repo_path():
    return os.path.realpath(os.path.dirname(os.path.realpath(__file__)) + "/../..")

def run_cpplint(filename, cpplint_path):
    cpplint = imp.load_source('cpplint', cpplint_path)
    cpplint._cpplint_state.ResetErrorCounts()
    cpplint.print_stdout = False
    cpplint._line_length = 120
    cpplint.output = []
    try:
        index = filename.split("/").index("include")
        "/".join(filename.split("/")[:index+1])
    except ValueError:
        pass
    cpplint.ProcessFile(filename, cpplint._cpplint_state.verbose_level)
    return cpplint.output

def print_objection():
    print "   ____  __      _           __  _             __"
    print "  / __ \/ /_    (_)__  _____/ /_(_)___  ____  / /"
    print " / / / / __ \  / / _ \/ ___/ __/ / __ \/ __ \/ / "
    print "/ /_/ / /_/ / / /  __/ /__/ /_/ / /_/ / / / /_/  "
    print "\____/_.___/_/ /\___/\___/\__/_/\____/_/ /_(_)   "
    print "          /___/                                  "

def main():
    num_errors = 0

    cpplint_path = get_cpplint_path()
    repo_path = get_repo_path()

    # Lets look for source files and headers in our repo
    for root, dirnames, filenames in os.walk(get_repo_path()):
        if "Software" in root or "external" in root or "gnc/matlab" in root or \
        "submodules" in root:
            continue
        for filename in filenames:
            if "agast_score" in filename or "brisk" in filename:
                continue
            if not filename.endswith((".cpp",".cc",".h",".hpp",".cc.in",".c.in",".h.in",
                                      ".hpp.in",".cxx",".hxx")):
                continue
            output = run_cpplint(root + "/" + filename, cpplint_path)
            # Print an objection at first sight of errors
            if num_errors == 0 and len(output) > 0:
                print_objection()
            lines = []
            num_errors += len(output)
            for error in output:
                print "%s:%s: %s" % ((root + "/" + filename).replace(get_repo_path() + "/", ''), str(error[0]), error[1])
                lines.append(error[0])
            # Run the clang-format if there are errors in the identified ranges
            if len(output) > 0:
                command = "clang-format-8 -style=file -i " + root + "/" + filename
                ranges = []
                for k, g in groupby(enumerate(lines), lambda (i,x):i-x):
                    group = map(itemgetter(1), g)
                    ranges.append((group[0], group[-1]))
                for clang_range in ranges:
                    command = command + " --lines=" + str(clang_range[0]) + ":" + str(clang_range[1])
                print command
                retcode = subprocess.Popen(command, shell=True)

    print "="*50
    if num_errors > 0:
        print "  You have %d lint errors" % num_errors
        print "  The errors were automatically corrected"
    elif num_errors == 0:
        print "  Code adheres to style guide lines"

    exit(num_errors)


if __name__ == "__main__":
  main()
