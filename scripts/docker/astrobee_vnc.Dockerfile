# On top of the normal Astrobee image, this will set up a VNC server
# running inside the Docker container so you can run graphical
# applications and view them using a VNC client.

ARG UBUNTU_VERSION=20.04
ARG REMOTE=astrobee
FROM ${REMOTE}/astrobee:latest-ubuntu${UBUNTU_VERSION}

# Rationale for packages:
#   xvfb: X server that doesn't need GPU or physical display
#   x11vnc: VNC server that shares xvfb display with external VNC clients
#   xfce4: lightweight window manager
#   dbus-x11: inter-process communication needed by xfce
#   x11-apps: X11 apps to demo such as xeyes
#   x11-utils: debugging commands such as xdpyinfo
#   x11-xserver-utils: setup commands such as xset

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    dbus-x11 \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    x11vnc \
    xfce4 \
    xvfb \
  && rm -rf /var/lib/apt/lists/*

COPY ./scripts/start_vnc.sh /start_vnc.sh

CMD ["/start_vnc.sh"]

# Expose VNC server's port
EXPOSE 5900
