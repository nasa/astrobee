ARG UBUNTU_VERSION=20.04
ARG REMOTE=ghcr.io/nasa
FROM ${REMOTE}/astrobee:latest-ubuntu${UBUNTU_VERSION}
ENV DEBIAN_FRONTEND=dialog

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
