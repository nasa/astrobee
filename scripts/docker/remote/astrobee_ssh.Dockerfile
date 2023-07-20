# On top of the normal Astrobee image, this will set up a ssh server
# running inside the Docker container so you can run graphical
# applications and view them.

ARG UBUNTU_VERSION=20.04
ARG REMOTE=astrobee
ARG IMAGE=${REMOTE}/astrobee:latest-ubuntu${UBUNTU_VERSION}
FROM ${IMAGE}

# Install dependencies
RUN apt-get update \
  && apt-get install -y \
    openssh-server \
    xauth \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir /var/run/sshd \
  && echo 'root:astrobee' | chpasswd \
  && sed -i 's/#X11UseLocalhost yes/X11UseLocalhost no/; s/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config \
  && echo "PermitEmptyPasswords yes" >> /etc/ssh/sshd_config

EXPOSE 22
CMD ["/usr/sbin/sshd", "-D"]