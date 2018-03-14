#/bin/bash

# Define the path to the plantuml library
# currently just assume that it has been copied locally
plantuml_jar = ./plantuml/plantuml.jar

# Path where the generated diagrams will go
gen_dir = ./exported

#uml_src = $(wildcard *.puml)
uml_src = OperatingState.puml MobilityState.puml PlanExecutionState.puml \
		EpsState.puml deployment.puml network.puml architecture.puml notations.puml

uml_svg = $(uml_src:%.puml=$(gen_dir)/%.svg)

uml_png = $(uml_src:%.puml=$(gen_dir)/%.png)

plantuml_ver = 1.2017.18
plantuml_zip = plantuml-jar-asl-$(plantuml_ver).zip
plantuml_url = https://sourceforge.net/projects/plantuml/files/$(plantuml_ver)/

all: svg

svg: $(plantuml_jar) $(uml_svg)

png: $(plantuml_jar) $(uml_png)

$(plantuml_jar):
	@echo "Missing plantuml jar: downloading now..."
	@mkdir ./plantuml
	@cd plantuml
	@wget $(plantuml_url)/$(plantuml_zip)
	@unzip -d plantuml $(plantuml_zip)
	@rm $(plantuml_zip)
	@echo "Got plantuml.jar!"

$(gen_dir)/%.svg: %.puml
	 @set -e; [ -d $(gen_dir) ] || mkdir $(gen_dir)
	 java -jar $(plantuml_jar) -tsvg -o $(gen_dir) $<

$(gen_dir)/%.png: %.puml
	 @set -e; [ -d $(gen_dir) ] || mkdir $(gen_dir)
	 java -jar $(plantuml_jar) -tpng -o $(gen_dir) $<

help:
	@echo "Create UML diagrams. Valid targets are:"
	@echo "  svg - create SVG version of the diagrams (default)"
	@echo "  png - create PNG version of the diagrams"
	@echo "  debug - print some information and exit"

debug:
	@echo "plantuml_jar = $(plantuml_jar)"
	@echo "gen_dir = $(gen_dir)"
	@echo "uml_src = $(uml_src)"
	@echo "uml_svg = $(uml_svg)"

.PHONY: all help debug
