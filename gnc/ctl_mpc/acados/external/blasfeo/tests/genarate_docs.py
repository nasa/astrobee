#! /usr/bin/python

import subprocess
import sys
import json
from jinja2 import Template

class RoutinesLibrary:
    def __init__(self, routines="routines.json"):

        with open(routines) as f:
            self.routines = json.load(f)


    def generate_docs(self, output_file="routines_list.md", template="routines_list.md.tpl"):

        with open(template) as f:
            template_txt = f.read()

        self.preprocess()
        template = Template(template_txt)
        output = template.render(routines=self.routines)
        with open(output_file, "w") as f:
            f.write(output)
        print(output)

    def preprocess(self):
        for rtn_class_name, rtn_class in self.routines.items():
            if rtn_class_name=="aux": self.build_subclasses("aux", skip=["mem"])
            self.add_precision(rtn_class_name)

    def build_subclasses(self, rtn_class, skip=[]):
        built_class = {}
        for rtn_subclass_name, rtn_subclass in self.routines[rtn_class].items():

            if rtn_subclass_name in skip:
                built_class.update(rtn_subclass)
                continue
            for rtn_name, rtn_info in rtn_subclass.items():
                built_class[f"{rtn_subclass_name}{rtn_name}"] = rtn_info

        self.routines[rtn_class] = built_class

    def add_precision(self, rtn_class):
        built_class = {}
        for rtn_name, rtn_info in self.routines[rtn_class].items():

            if "{PREC}" not in rtn_name and "STR" not in rtn_name:
                # default naming
                routine_name = f"blasfeo_(s|d){rtn_name}".replace("_", "\_")
            else:
                # templated naming
                routine_name="blasfeo_"+rtn_name.format(
                    PREC="(s|d)",
                    STRVEC="(s|d)vec",
                    STRMAT="(s|d)mat")
                routine_name = routine_name.replace("_", "\_")

            built_class[routine_name] = rtn_info


        self.routines[rtn_class] = built_class


if __name__ == "__main__":

    # generate recipes
    # test set to be run in the given excution of the script
    routines_lib = RoutinesLibrary()
    #  print(json.dumps(cookbook.recipe, indent=4))
    routines_lib.generate_docs()
