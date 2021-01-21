# This will set up an Astrobee melodic docker container using the non-NASA install
# instructions.
# This image is the base, meaning that it contains all the installation context,
# but it doesn't copy or build the entire code.
# You must set the docker context to be the repository root directory

FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04

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
  && rm -rf /var/lib/apt/lists/*

# suppress detached head warnings later
RUN git config --global advice.detachedHead false

# Install ROS Melodic----------------------------------------------------------------
COPY ./scripts/setup/*.sh /setup/astrobee/

# this command is expected to have output on stderr, so redirect to suppress warning
RUN /setup/astrobee/add_ros_repository.sh >/dev/null 2>&1

RUN apt-get update \
  && apt-get install -y \
  debhelper \
  libtinyxml-dev \
  ros-melodic-desktop \
  python-rosdep \
  && rm -rf /var/lib/apt/lists/*

# Install OpenCV----------------------------------------------------------------
# Install depending packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    openexr \
    libatlas-base-dev \
    python3-dev \
    python3-numpy \
    libtbb2 \
    libtbb-dev \
    libdc1394-22-dev \
  && rm -rf /var/lib/apt/lists/*

# Downloading OpenCV repo & switching to 3.3.1 branch,
# Building OpenCV
RUN mkdir /opencv_build && cd /opencv_build && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout 3.3.1 && cd .. && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && git checkout 3.3.1 && \
    cd /opencv_build/opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=${install_path:-/usr/local} \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D OPENCV_EXTRA_MODULES_PATH=/opencv_build/opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON \
      -D OPENCV_ENABLED_NONFREE=YES \
      -D ENABLE_PRECOMPILED_HEADERS=OFF .. && \
    make -j6
RUN cd /opencv_build/opencv/build  \
  && make install \
  && rm -rf /opencv_build \
  && rm -rf /var/lib/apt/lists/*

# Install Astrobee----------------------------------------------------------------
COPY ./scripts/setup/debians /setup/astrobee/debians

RUN apt-get update \
  && /setup/astrobee/install_luajit.sh \
  && /setup/astrobee/debians/build_install_debians_18_04.sh \
  && rm -rf /var/lib/apt/lists/*

COPY ./scripts/setup/packages_*.lst /setup/astrobee/
# note apt-get update is run within the following shell script
RUN /setup/astrobee/install_desktop_18_04_packages.sh \
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
RUN echo "#!/bin/bash\nset -e\n\nsource \"/opt/ros/melodic/setup.bash\"\nsource \"/build/astrobee/devel/setup.bash\"\nexport ASTROBEE_CONFIG_DIR=\"/src/astrobee/astrobee/config\"\nexec \"\$@\"" > /astrobee_init.sh && \
  chmod +x /astrobee_init.sh && \
  rosdep init && \
  rosdep update 2>&1 | egrep -v 'as root|fix-permissions'