# This will set up an Astrobee kinetic docker container using the non-NASA install
# instructions.
# This image is the base, meaning that it contains all the installation context,
# but it doesn't copy or build the entire code.
# This image builds on top of the base kinetic image building the code.

# You must set the docker context to be the repository root directory

FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04

# try to suppress certain warnings during apt-get calls
ARG DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# install of apt-utils suppresses bogus warnings later
RUN apt-get update \
  && apt-get install -y apt-utils 2>&1 | grep -v "debconf: delaying package configuration, since apt-utils is not installed" \
  && apt-get install -y \
    build-essential \
    git \
    lsb-release \
    sudo \
    wget \
    curl \
  && rm -rf /var/lib/apt/lists/*
COPY ./scripts/setup/*.sh /setup/astrobee/
COPY ./scripts/setup/debians /setup/astrobee/debians

# this command is expected to have output on stderr, so redirect to suppress warning
RUN /setup/astrobee/add_ros_repository.sh >/dev/null 2>&1

RUN apt-get update && apt-get install -y \
  libtinyxml-dev \
  ros-kinetic-opencv3 \
  && rm -rf /var/lib/apt/lists/*

# suppress detached head warnings later
RUN git config --global advice.detachedHead false

RUN apt-get update \
  && /setup/astrobee/debians/build_install_debians.sh \
  && rm -rf /var/lib/apt/lists/*

COPY ./scripts/setup/packages_*.lst /setup/astrobee/
# note apt-get update is run within the following shell script
RUN /setup/astrobee/install_desktop_packages.sh \
  && rm -rf /var/lib/apt/lists/*

#Add new sudo user
ENV USERNAME astrobee
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME

#Add the entrypoint for docker
RUN echo "#!/bin/bash\nset -e\n\nsource \"/opt/ros/kinetic/setup.bash\"\nsource \"/build/astrobee/devel/setup.bash\"\nexport ASTROBEE_CONFIG_DIR=\"/src/astrobee/astrobee/config\"\nexec \"\$@\"" > /astrobee_init.sh && \
  chmod +x /astrobee_init.sh && \
  rosdep init && \
  rosdep update 2>&1 | egrep -v 'as root|fix-permissions'