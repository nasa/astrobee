# This will set up an Astrobee melodic docker container using the non-NASA
# install instructions. This image builds on top of the base melodic image
# building the code. You must set the docker context to be the repository root
# directory
#
# This dockerfile is an alternative to `astrobee.Dockerfile`. It uses previous
# docker image build artifacts to speed up the catkin build step. If, in the
# future, you were to use a 'quick' astrobee's image as a build cache, be sure
# to add `catkin clean --orphans` so you don't bloat the cache image down the
# line.

# This image generates documentation for the Astrobee project

FROM ubuntu:focal

ENV DEBIAN_FRONTEND=noninteractive

# Set up time zone to avoid any prompts during install later
RUN echo 'Etc/UTC' > /etc/timezone \
  && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

# Install dependencies
RUN apt-get update \
  && apt-get install -y \
    bison \
    build-essential \
    cmake \
    doxygen \
    flex \
    git \
    graphviz \
    openjdk-8-jdk \
    python3-iso8601 \
    python3-pip \
    python3-setuptools \
    python3-six \
    unzip \
    wget

# Set up so scripts starting with "#!/usr/bin/env python" run under python3 in focal
RUN which python >/dev/null || update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Install xgds_planner
RUN git clone --quiet --branch just_xpjson https://github.com/trey0/xgds_planner2.git \
    && cd xgds_planner2 \
    && python setup.py install

# Copy over the repo
COPY . /repo

# Generate command line dictionary
RUN cd /repo \
    && mkdir -p doc/html \
    && ./scripts/build/genCommandDictionary.py astrobee/commands/freeFlyerPlanSchema.json doc/html/AstrobeeCommandDictionary.html

# Build Documentation
RUN cd /repo/doc/diagrams/ \
    && make \
    && cd ../.. \
    && doxygen astrobee.doxyfile
