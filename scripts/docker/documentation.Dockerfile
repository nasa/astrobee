# This will set up an Astrobee melodic docker container using the non-NASA install instructions.
# This image builds on top of the base melodic image building the code.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=18.04
FROM ubuntu:${UBUNTU_VERSION}

# Install dependencies
RUN apt-get update \
  && apt-get install -y \
  build-essential \
  cmake \
  bison \
  flex \
  graphviz \
  wget \
  git \
  unzip \
  python-pip \
  python-setuptools \
  openjdk-8-jdk

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
RUN pip install iso8601 \
    && pip install "django<2" \
    && git clone https://github.com/geocam/geocamUtilWeb.git \
    && cd geocamUtilWeb \
    && python setup.py install \
    && cd .. \
    && git clone https://github.com/xgds/xgds_planner2.git \
    && cd xgds_planner2 \
    && python setup.py install

# Copy over the repo
COPY . /repo

# Generate command line dictionary
RUN cd /repo \
    && mkdir -p doc/html \
    && ./scripts/build/genCommandDictionary.py astrobee/commands/freeFlyerPlanSchema.json doc/html/AstrobeeCommandDictionary.html

# Build Documentation
RUN cd /repo/doc/diagrams/ && make && cd ../.. \
    && doxygen astrobee.doxyfile