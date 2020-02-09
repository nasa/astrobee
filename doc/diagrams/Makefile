#/bin/bash

# Define the path to the plantuml library
# currently just assume that it has been copied locally
plantuml_jar = ./plantuml/plantuml.jar

# Path where the generated diagrams will go
gen_dir = ./gen
svg_dir = $(gen_dir)/svg
png_dir = $(gen_dir)/png
eps_dir = $(gen_dir)/eps
pdf_dir = $(gen_dir)/pdf

#uml_src = $(wildcard *.puml)
uml_src = OperatingState.puml MobilityState.puml PlanExecutionState.puml \
		EpsState.puml FaultSequences.puml overview-horizontal.puml \
		nodes_overview.puml nodes_interactions.puml \
		deployment.puml network.puml architecture.puml gnc_overview.puml \
		sim_concepts_robot.puml sim_concepts_simulator.puml ars_overview.puml\
		notations.puml llp_nodes.puml mlp_nodes.puml hlp_nodes.puml \
		packages_linux.puml packages_android.puml color_palettes.puml

uml_svg = $(uml_src:%.puml=$(svg_dir)/%.svg)
uml_png = $(uml_src:%.puml=$(png_dir)/%.png)
uml_eps = $(uml_src:%.puml=$(eps_dir)/%.eps)
uml_pdf = $(uml_src:%.puml=$(pdf_dir)/%.pdf)

dot_src = $(wildcard *.dot)
dot_svg = $(dot_src:%.dot=$(svg_dir)/%.svg)
dot_png = $(dot_src:%.dot=$(png_dir)/%.png)

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

# Get the source code root directory
make_dir=$(shell dirname $(lastword $(MAKEFILE_LIST)))
ars_root_dir=$(realpath $(make_dir)/../..)

all: svg

doc: svg
	cd $(ars_root_dir) && SOURCE_PATH=$(ars_root_dir) doxygen freeflyer.doxyfile

jar: $(plantum_jar)

svg: $(plantuml_jar) $(uml_svg) $(dot_svg)

png: $(plantuml_jar) $(uml_png) $(dot_png)

eps: $(plantuml_jar) $(uml_eps)

pdf: $(plantuml_jar) $(uml_pdf)

$(plantuml_jar):
	@echo "Missing plantuml jar: downloading now..."
	@mkdir ./plantuml
	@cd plantuml
	@wget $(plantuml_url)/$(plantuml_zip)
	@unzip -d plantuml $(plantuml_zip)
	@rm $(plantuml_zip)
	@echo "Got plantuml.jar!"

$(svg_dir)/%.svg: %.puml
	 @echo "Generating diagram $@"
	 @set -e; [ -d $(svg_dir) ] || mkdir -p $(svg_dir)
	 @java -jar $(plantuml_jar) -tsvg \
	 	-ScomponentStyle=uml2 \
	 	-SdefaultFontName="Trebuchet MS" \
		-SdefaultFontSize=9 \
		-o $(svg_dir) $<

$(eps_dir)/%.eps: %.puml
	 @echo "Generating diagram $@"
	 @set -e; [ -d $(eps_dir) ] || mkdir -p $(eps_dir)
	 @java -jar $(plantuml_jar) -teps \
	 	-ScomponentStyle=uml2 \
	 	-SdefaultFontName="Trebuchet MS" \
		-SdefaultFontSize=9 \
		-o $(eps_dir) $<

$(pdf_dir)/%.pdf: %.puml
	 @echo "Generating diagram $@"
	 @set -e; [ -d $(pdf_dir) ] || mkdir -p $(pdf_dir)
	 @java -jar $(plantuml_jar) -tpdf \
	 	-ScomponentStyle=uml2 \
		-o $(pdf_dir) $<

$(png_dir)/%.png: %.puml
	 @echo "Generating diagram $@"
	 @set -e; [ -d $(png_dir) ] || mkdir -p $(png_dir)
	 @java -DPLANTUML_LIMIT_SIZE=$(max_memory) -jar $(plantuml_jar) -tpng \
	 	-ScomponentStyle=uml2 \
	  -Sdpi=300 \
	 	-o $(png_dir) $<

$(svg_dir)/%.svg: %.dot
	 @set -e; [ -d $(svg_dir) ] || mkdir -p $(svg_dir)
	 dot -Tsvg -o $@ $<

$(png_dir)/%.png: %.dot
	 @set -e; [ -d $(png_dir) ] || mkdir -p $(png_dir)
	 dot -Tpng -Gdpi=300 -o $@ $<

help:
	@echo "Create UML diagrams. Valid targets are:"
	@echo "  svg - create SVG version of the diagrams (default)"
	@echo "  png - create PNG version of the diagrams"
	@echo "  jar - Just get the PlanUML jar file"
	@echo "  debug - print some information and exit"

debug:
	@echo "ars_root_dir = $(ars_root_dir)"
	@echo "plantuml_jar = $(plantuml_jar)"
	@echo "gen_dir = $(gen_dir)"
	@echo "uml_src = $(uml_src)"
	@echo "uml_svg = $(uml_svg)"
	@echo "uml_png = $(uml_png)"
	@echo "dot_src = $(dot_src)"
	@echo "dot_svg = $(dot_svg)"
	@echo "dot_png = $(dot_png)"

.PHONY: all doc help debug
