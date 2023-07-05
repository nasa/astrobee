FROM ghcr.io/nasa/astrobee:latest-rolling-ubuntu20.04
ENV DEBIAN_FRONTEND=dialog

RUN apt-get update \
  && apt-get install -y \
  libgoogle-glog-dev libgflags-dev libgtest-dev \
  libluajit-5.1-dev \
  ros-rolling-xacro \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir /src && mkdir /src/astrobee
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
    -t robbyrussell
RUN echo source /opt/ros/rolling/setup.zsh >> /root/.zshrc
RUN echo source /src/astrobee/install/setup.zsh >> /root/.zshrc
CMD [ "zsh" ]
ENV MAKEFLAGS="-j4"