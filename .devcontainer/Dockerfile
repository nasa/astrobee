FROM ghcr.io/nasa/astrobee:latest-rolling-ubuntu20.04
ENV DEBIAN_FRONTEND=dialog

RUN apt-get update \
  && apt-get install -y \
  libgoogle-glog-dev libgflags-dev libgtest-dev \
  libluajit-5.1-dev \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir /src && mkdir /src/astrobee