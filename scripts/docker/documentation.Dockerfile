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

# This image generates doxygen documentation for the Astrobee project

ARG UBUNTU_VERSION=20.04
FROM ubuntu:${UBUNTU_VERSION}

# Install dependencies
RUN apt-get update \
  && apt-get install -y \
  bison \
  build-essential \
  cmake \
  flex \
  git \
  graphviz \
  openjdk-8-jdk \
  python-pip \
  python-setuptools \
  unzip \
  wget

# Install doxygen
RUN wget 'https://sourceforge.net/projects/doxygen/files/rel-1.8.20/doxygen-1.8.20.src.tar.gz' \
    && tar -zxvf doxygen-1.8.20.src.tar.gz \
    && cd doxygen-1.8.20 \
    && mkdir build \
    && cd build \
    && cmake -G "Unix Makefiles" .. \
    && make \
    && make install

# Install xgds_planner
RUN pip3 install \
      iso8601 \
      six \
    && git clone --quiet --branch just_xpjson https://github.com/trey0/xgds_planner2.git \
    && cd xgds_planner2 \
    && python3 setup.py install

# Copy over the repo
COPY . /repo

# Generate command line dictionary
RUN cd /repo \
    && mkdir -p doc/html \
    && python3 ./scripts/build/genCommandDictionary.py astrobee/commands/freeFlyerPlanSchema.json doc/html/AstrobeeCommandDictionary.html

# Build Documentation
RUN cd /repo/doc/diagrams/ \
    && make \
    && cd ../.. \
    && doxygen astrobee.doxyfile
