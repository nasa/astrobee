#! /usr/bin/python

import subprocess as sp
import jinja2 as jn
import sys
import json
import argparse
import re
from hashlib import sha1
from pathlib import Path
from collections import OrderedDict
import shutil
import multiprocessing



BLASFEO_PATH=str(Path(__file__).absolute().parents[1])
BLASFEO_TEST_PATH=str(Path(__file__).absolute().parents[0])

TEST_SCHEMA="test_schema.json"
TESTSET_JSON="testset_default.json"
BUILDS_DIR="build"
REPORTS_DIR="reports"
TESTCLASSES_DIR="classes"
TPL_PATH="Makefile.tpl"
LIB_BLASFEO_STATIC = "libblasfeo.a"
#MAKE_FLAGS={"--jobs":"8"}
MAKE_FLAGS={"--jobs": multiprocessing.cpu_count()}
SILENT=0



def parse_arguments():
	parser = argparse.ArgumentParser(description='BLASFEO tests scheduler')

	parser.add_argument(dest='testset_json', type=str, default=TESTSET_JSON, nargs='?',
		help='Run a batch of test from a specific testset, i.e. testset_all.json')
	parser.add_argument('--silent', default=False, action='store_true',
		help='Silent makefile output')
	parser.add_argument('--continue', dest='continue_test', default=False, action='store_true',
		help='Do not interrupt tests sweep on error')
	parser.add_argument('--rebuild', default=False, action='store_true',
		help='Rebuild libblasfeo to take into account recent code '+
		'changes or addition of new target to the testset batch')

	args = parser.parse_args()
	return args



def run_cmd(cmd, stdinput=""):

	if not SILENT: print(cmd,"\n")

	cmd_proc = sp.Popen(cmd,
		shell=True,
		stdin=sp.PIPE,
		stdout=sp.PIPE,
		stderr=sp.PIPE
		)

	outs, errs = cmd_proc.communicate(stdinput.encode("utf8"))

	if outs and not SILENT: print("Make infos:\n{}".format(outs.decode("utf8")))
	if errs: print("Make errors:\n{}".format(errs.decode("utf8")))

	return cmd_proc



def make_blasfeo(cmd="", env_flags={}, blasfeo_flags={}):

	# blasfeo compilation flags
	blasfeo_flags_str = " ".join([
		"{k}={v}".format(k=k, v=v)
		if v is not None else "{k}".format(k=k)
		for k, v in blasfeo_flags.items()
	])

	# enviroment flags
	env_flags_str = " ".join([
		"{k}={v}".format(k=k, v=v)
		for k, v in env_flags.items()
	])

	# compiler flags
	make_flags_str = " ".join([
		"{k}={v}".format(k=k, v=v)
		if v is not None else "{k}".format(k=k)
		for k, v in MAKE_FLAGS.items()
	])

	# clean build folder
	clean_cmd = "make deep_clean -C .. "
	cmd_proc = run_cmd(clean_cmd)

	# make blasfeo with selected flag set
	make_cmd = "{env_flags} make {blasfeo_flags} {make_flags} -C .. static_library"\
		.format(env_flags=env_flags_str,
				blasfeo_flags=blasfeo_flags_str,
				make_flags=make_flags_str, cmd=cmd)
	cmd_proc = run_cmd(make_cmd)

	return cmd_proc.returncode



class BlasfeoTestset:
	def __init__(self, cli_flags):
		global SILENT

		self.cli_flags=cli_flags
		self.continue_test = 0

		self.scheduled_routines = {}
		self.test_routines = {}

		with open(cli_flags.testset_json) as f:
			self.specs = json.load(f, object_pairs_hook=OrderedDict)

		with open(TPL_PATH) as f:
			self.makefile_template = jn.Template(f.read())

		if self.specs["options"].get("silent") or self.cli_flags.silent:
			SILENT = 1

		if self.specs["options"].get("continue") or  self.cli_flags.continue_test:
			self.continue_test = 1


		with open(TEST_SCHEMA) as f:
			self.schema = json.load(f, object_pairs_hook=OrderedDict)

		self._success_n = 0
		self._errors_n = 0

		self._total_n =\
			len(set(self.specs["routines"]))\
			* len(self.specs["TARGET"])\
			* len(self.specs["K_MAX_STACK"])\
			* len(self.specs["PACKING_ALG"])\
			* len(self.specs["precisions"])\
			* len(self.specs["apis"])

		# build standard testset skelethon
		self.build_testset()

	def parse_routine_options(self, routine_name, available_flags):

		# routine without any flag
		if not available_flags:
			return {'routine_basename': routine_name}

		pattern = '(?P<routine_basename>[a-z]*)_'

		for flag_name, flags_values in available_flags.items():
			flags_values = '|'.join(flags_values)
			pattern += '(?P<{flag_name}>[{flags_values}])'.format(flag_name=flag_name, flags_values=flags_values)

		parsed_flags = re.search(pattern, routine_name)

		if not parsed_flags:
			print("Error parsing flags of routine: {routine_name}".format(routine_name=routine_name))
			return {}

		return parsed_flags.groupdict()

	def build_testset(self):
		scheduled_routines = set(self.specs['routines'])

		# create testset with no global flags
		self.testset = OrderedDict(self.specs)
		self.testset["scheduled_routines"] = {}

		available_groups = self.schema['routines']

		# routine groups: blas1 blas2 ..
		for group_name, available_classes in available_groups.items():

			# routine classes: gemm, trsm, ...
			for class_name, routine_class in available_classes.items():
				available_routines = routine_class["routines"]
				routine_flags = routine_class["flags"]

				# routines: gemm, gemm_nn, gemm_nt, ...
				for routine  in available_routines:

					if routine not in scheduled_routines:
						continue

					scheduled_routines = scheduled_routines - {routine}

					# precision
					for precision in self.specs["precisions"]:

						# apis
						for api in self.specs["apis"]:

							test_macros = {}

							test_macros["ROUTINE_CLASS"] = class_name
							test_macros["PRECISION_{}".format(precision.upper())] = None

							routine_fullname = "{api}_{precision}{routine}".format(
								api=api, precision=precision[0], routine=routine)

							if api=="blas":
								test_macros["TEST_BLAS_API"] = None
								routine_dict = self.parse_routine_options(routine, routine_flags)
								if not routine_dict: continue
								test_macros.update(routine_dict)
								routine_testclass_src = "blasapi_"+routine_class["testclass_src"]
								routine_name = "{precision}{routine}".format(precision=precision[0], routine=class_name)
							else:
								routine_testclass_src = routine_class["testclass_src"]
								routine_name = "{precision}{routine}".format(precision=precision[0], routine=routine)

							test_macros["ROUTINE_CLASS_C"] = str(Path(TESTCLASSES_DIR, routine_testclass_src))
							test_macros["ROUTINE"] = routine_name
							test_macros["ROUTINE_FULLNAME"] = routine

							# add blas_api flag arguments values

							self.testset["scheduled_routines"][routine_fullname] = {
								"group": group_name,
								"class": class_name,
								"api": api,
								"precision": precision,
								"make_cmd": "update",
								"fullname": routine_fullname,
								"test_macros": test_macros
							}

		if scheduled_routines:
			print("Some routines were not found in the schema ({}) {}"
				  .format(TEST_SCHEMA, scheduled_routines))

	def run_all(self):
		# tune the testset and run

		for la in self.specs["LA"]:
			self.testset["blasfeo_flags"]["LA"]=la
			self.testset["test_macros"]["BLASFEO_LA"]=la

			if la=="REFERENCE":
				self.run_testset()
				break

			if la=="EXTERNAL_BLAS_WRAPPER":
				self.run_testset()
				break

			for mf in self.specs["MF"]:
				self.testset["blasfeo_flags"]["MF"]=mf
				self.testset["test_macros"]["BLASFEO_MF"]=mf

				for target in self.specs["TARGET"]:
					self.testset["blasfeo_flags"]["TARGET"]=target
					self.testset["test_macros"]["BLASFEO_TARGET"]=target

					for max_stack in self.specs["K_MAX_STACK"]:
						self.testset["blasfeo_flags"]["K_MAX_STACK"]=max_stack

						print("\n## Testing {la}:{target} kswitch={max_stack}".format(target=target, la=la, max_stack=max_stack))

						for alg in self.specs["PACKING_ALG"]:
							self.testset["blasfeo_flags"]["PACKING_ALG"]=alg
							self.testset["test_macros"]["PACKING_ALG"]=alg

							self.run_testset()

	def is_lib_updated(self):

		target = self.testset["blasfeo_flags"]["TARGET"]
		la = self.testset["blasfeo_flags"]["LA"]

		if self.lib_static_dst.is_file():
			return 1

		return 0

	def run_testset(self):
		# preparation step
		test_macros = self.testset["test_macros"]
		blasfeo_flags = self.testset["blasfeo_flags"]
		env_flags = self.testset["env_flags"]

		# always compile blas api
#		blasfeo_flags.update({"BLAS_API":1})
		blasfeo_flags.update({"BLASFEO_PATH":str(BLASFEO_PATH)})

		blasfeo_flags_str = "_".join(
			["{k}={v}".format(k=k, v=v) if v is not None else '' for k, v in blasfeo_flags.items()])

		# create unique folder name from set of flags
		m = sha1(blasfeo_flags_str.encode("utf8"))
		binary_dir = m.hexdigest()
		# binary path for current_conf build
		binary_path = Path(BLASFEO_TEST_PATH, BUILDS_DIR, binary_dir)
		# create directory
		binary_path.mkdir(parents=True, exist_ok=True)
		# update binary_dir flag
		blasfeo_flags.update({"ABS_BINARY_PATH":str(binary_path)})

		self.lib_static_src = Path(BLASFEO_PATH, "lib", LIB_BLASFEO_STATIC)

		self.lib_static_dst = Path(binary_path, LIB_BLASFEO_STATIC)

		lib_flags_json = str(Path(binary_path, "flags.json"))

		if self.cli_flags.silent or self.testset["options"].get("silent"):
			MAKE_FLAGS.update({"-s":None})

		if self.cli_flags.rebuild or self.testset["options"].get("rebuild") or not self.is_lib_updated():
			# compile the library
			make_blasfeo(blasfeo_flags=blasfeo_flags, env_flags=env_flags)

			# write used flags
			with open(lib_flags_json, "w") as f:
				json.dump(blasfeo_flags, f, indent=4)

			# copy library
			shutil.copyfile(str(self.lib_static_src), str(self.lib_static_dst))

			#  self.lib_static_dst.write_bytes(lib_static_src.read_bytes())

		for routine_name, args in self.testset['scheduled_routines'].items():
			# update local flags with global flags

			if args.get("test_macros"):
				args["test_macros"].update(test_macros)
			else:
				args["test_macros"] = test_macros

			if self.continue_test:
				args["test_macros"].update({"CONTINUE_ON_ERROR":1})

			if args.get("env_flags"):
				args["env_flags"].update(env_flags)
			else:
				args["env_flags"] = env_flags

			if args.get("blasfeo_flags"):
				args["blasfeo_flags"].update(blasfeo_flags)
			else:
				args["blasfeo_flags"] = blasfeo_flags

			error =  self.run_routine(routine_name, args)

			if error and not self.continue_test:
				break

	def render_routine(self, make_cmd="", env_flags={}, test_macros={}, blasfeo_flags={},
					   **kargs):

		# compress run_id
		la=blasfeo_flags['LA']
		if la != "HIGH_PERFORMANCE": target=la

		run_id = "{target}_{routine_fullname}_stack{kstack}_pack{alg}"\
			.format(
				target=blasfeo_flags['TARGET'],
				routine_fullname=kargs["fullname"],
				kstack=blasfeo_flags['K_MAX_STACK'],
				alg=blasfeo_flags['PACKING_ALG']
				)

		print("Testing {run_id}".format(run_id=run_id))

		makefile = self.makefile_template.render(test_macros=test_macros)

		# flag to override default libblasfeo flags
		blasfeo_flags_cmd = " ".join(["{k}={v}"\
			.format(k=k, v=v) if v is not None else k for k, v in blasfeo_flags.items()])
		make_flags = " ".join(["{k}={v}"\
			.format(k=k, v=v) if v is not None else k for k, v in MAKE_FLAGS.items()])

		make_cmd = "make {blasfeo_flags_cmd} -f - {make_cmd}"\
			.format(make_cmd=make_cmd, blasfeo_flags_cmd=blasfeo_flags_cmd)

		# render command to write on reports folder for later inspection
		report_cmd = "make {blasfeo_flags_cmd} {make_flags} {make_cmd}"\
			.format(make_cmd=make_cmd,
					make_flags=make_flags,
					blasfeo_flags_cmd=blasfeo_flags_cmd)

		# add entry in tested_routine
		self.test_routines[run_id] = {}
		self.test_routines[run_id]["make_cmd"] = make_cmd
		self.test_routines[run_id]["report_cmd"] = report_cmd
		self.test_routines[run_id]["makefile"] = makefile

		return run_id

	def run_routine(self, routine_fullname, kargs):

		run_id = self.render_routine(**kargs)

		make_cmd = self.test_routines[run_id]["make_cmd"]
		report_cmd = self.test_routines[run_id]["report_cmd"]
		makefile = self.test_routines[run_id]["makefile"]

		cmd_proc = run_cmd(make_cmd, stdinput=makefile)

		if cmd_proc.returncode:
			self._errors_n += 1
			self.test_routines[run_id]["success"]=0

			# on error write report for future inspection
			report_path = Path(REPORTS_DIR, run_id)
			report_path.mkdir(parents=True, exist_ok=True)
			with open(str(Path(report_path, "Makefile")), "w") as f:
				f.write(makefile)
			with open(str(Path(report_path, "make.sh")), "w") as f:
				f.write("#! /bin/bash\n")
				f.write(report_cmd+"\n")

			print("Error with {run_id}".format(run_id=run_id))

		else:
			self.test_routines[run_id]["success"]=1
			self._success_n += 1
			if not SILENT: print("Tested with {run_id}".format(run_id=run_id))

		# print partial
		if not SILENT: print("({done}:Succeded, {errors}:Errors) / ({total}:Total)\n"
			.format(done=self._success_n, errors=self._errors_n, total=self._total_n))

		return cmd_proc.returncode

	def print_summary(self):

		print("\n\n### Testset Summary:\n")
		summary_lines = [
			"Error with {run_id}".format(run_id=run_id)
			for (run_id, values) in self.test_routines.items() if not values["success"]
		]

		print("\n".join(summary_lines))
		print("({done}:Succeded, {errors}:Errors) / ({total}:Total)\n"
			.format(done=self._success_n, errors=self._errors_n, total=self._total_n))
		print("See blasfeo/tests/reports to inspect specific errors")


	def get_returncode(self):
		if self._errors_n > 0:
			return 1
		return 0



if __name__ == "__main__":

	cli_flags = parse_arguments()

	# generate test set
	# collection of routines/lib combinations to be run in the given excution of the tester.py
	testset = BlasfeoTestset(cli_flags)
	#  print(json.dumps(testset.testset, indent=4))
	testset.run_all()
	testset.print_summary()

	sys.exit(testset.get_returncode())
