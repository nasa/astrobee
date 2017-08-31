#!/usr/bin/env python

import os, imp, fnmatch

num_errors = 0

def get_cpplint_path():
  return os.path.dirname(os.path.realpath(__file__)) + "/cpplint.py"

def get_repo_path():
  return os.path.realpath(os.path.dirname(os.path.realpath(__file__)) + "/..")

def run_cpplint(cpplint, filename):
  global num_errors
  cpplint._cpplint_state.ResetErrorCounts()
  cpplint.output = []
  try:
    index = filename.split("/").index("include")
    cpplint._root = os.path.relpath("/".join(filename.split("/")[:index+1]), get_repo_path())
  except ValueError:
    pass
  cpplint.ProcessFile(filename, cpplint._cpplint_state.verbose_level)

  if (num_errors == 0 and len(cpplint.output) > 0):
    print_objection()

  num_errors += len(cpplint.output)
  for error in cpplint.output:
    print "%s:%s: %s" % (filename, str(error[0]), error[1])

def print_objection():
    print "   ____  __      _           __  _             __"
    print "  / __ \/ /_    (_)__  _____/ /_(_)___  ____  / /"
    print " / / / / __ \  / / _ \/ ___/ __/ / __ \/ __ \/ / "
    print "/ /_/ / /_/ / / /  __/ /__/ /_/ / /_/ / / / /_/  "
    print "\____/_.___/_/ /\___/\___/\__/_/\____/_/ /_(_)   "
    print "          /___/                                  "

def main():
  # Load up cpplint
  cpplint = imp.load_source('cpplint', get_cpplint_path())
  cpplint._cpplint_state.ResetErrorCounts()
  cpplint.print_stdout = False
  cpplint._line_length = 120
  cpplint.output = []

  # Lets look for source files and headers in our repo
  for root, dirnames, filenames in os.walk(get_repo_path()):
    if "Software" in root or "external" in root or "gnc/matlab" in root or "submodules" in root:
      continue
    print filenames
    for filename in fnmatch.filter(filenames, "*.cc"):
      run_cpplint(cpplint, os.path.join(root, filename))
    for filename in fnmatch.filter(filenames, "*.h"):
      run_cpplint(cpplint, os.path.join(root, filename))

  print "="*50
  if num_errors > 0:
    print "  You have %d lint errors, commit failed" % num_errors
  elif num_errors == 0:
    print "  Code adheres to style guide lines"

  exit(num_errors)

if __name__ == "__main__":
  main()
