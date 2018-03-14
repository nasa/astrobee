#!/usr/bin/python

import argparse
import os
import os.path
import re
import subprocess
import sys
import yaml


def main():
	try:
		path = os.environ['BUILD_PATH']
	except KeyError:
		print >> sys.stderr, 'BUILD_PATH is not defined'
		return 0

	SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

	parser = argparse.ArgumentParser(description='Convert Alvar to Kalibr.')
	parser.add_argument('--config', default=SCRIPT_DIR + '/../../astrobee/config/dock_markers_specs.config', dest='config', help='The target configuration file with AR markers specs. Default = dock_markers_specs.config', required=False)
	args = parser.parse_args()

	#Setup Files
	target_file = SCRIPT_DIR + '/data/granite_april_tag.yaml'

	ret = subprocess.call(('./markers2Kalibr %s %s' % (args.config, target_file)).split(),
	cwd=path +'/bin', stdout=None, stderr=None)

if __name__ == '__main__':
  main()
