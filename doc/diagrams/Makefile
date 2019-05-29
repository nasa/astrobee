#/bin/bash

# Define the path to the plantuml library
# currently just assume that it has been copied locally
plantuml_jar = ./plantuml/plantuml.jar

# Path where the generated diagrams will go
gen_dir = ./exported

#uml_src = $(wildcard *.puml)
uml_src = OperatingState.puml MobilityState.puml PlanExecutionState.puml \
		EpsState.puml deployment.puml network.puml architecture.puml \
		sim_concepts_robot.puml sim_concepts_simulator.puml ars_overview.puml\
		notations.puml

uml_svg = $(uml_src:%.puml=$(gen_dir)/%.svg)

uml_png = $(uml_src:%.puml=$(gen_dir)/%.png)

dot_src = $(wildcard *.dot)
dot_svg = $(dot_src:%.dot=$(gen_dir)/%.svg)
dot_png = $(dot_src:%.dot=$(gen_dir)/%.png)

plantuml_ver = 1.2019.1
plantuml_zip = plantuml-jar-asl-$(plantuml_ver).zip
plantuml_url = https://sourceforge.net/projects/plantuml/files/$(plantuml_ver)/

ifndef BOOST_PLANTUML
#nb_threads=2
max_memory=8192
else
#nb_threads=4
# PlantUML option: -nbthread $(nb_threads)
# is useless in the makefile since each file is processed individually
# (no wildcard in the file to process)
# The correct way to parallelize the diagram generation is to use
# parallel make with the -j option (for example: make -j4)
max_memory=12284
endif

all: svg

jar: $(plantum_jar)

svg: $(plantuml_jar) $(uml_svg) $(dot_svg)

png: $(plantuml_jar) $(uml_png) $(dot_png)

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
	 java -DPLANTUML_LIMIT_SIZE=$(max_memory) -jar $(plantuml_jar) -tpng -o $(gen_dir) $<

$(gen_dir)/%.svg: %.dot
	 @set -e; [ -d $(gen_dir) ] || mkdir $(gen_dir)
	 dot -Tsvg -o $@ $<

$(gen_dir)/%.png: %.dot
	 @set -e; [ -d $(gen_dir) ] || mkdir $(gen_dir)
	 dot -Tpng -Gdpi=300 -o $@ $<

help:
	@echo "Create UML diagrams. Valid targets are:"
	@echo "  svg - create SVG version of the diagrams (default)"
	@echo "  png - create PNG version of the diagrams"
	@echo "  jar - Just get the PlanUML jar file"
	@echo "  debug - print some information and exit"

debug:
	@echo "plantuml_jar = $(plantuml_jar)"
	@echo "gen_dir = $(gen_dir)"
	@echo "uml_src = $(uml_src)"
	@echo "uml_svg = $(uml_svg)"
	@echo "dot_src = $(dot_src)"
	@echo "dot_svg = $(dot_svg)"
	@echo "dot_png = $(dot_png)"

.PHONY: all help debug
