
# Astrobee Docker container with graphical applications shared via VNC #

These setup instructions assume you want to run graphical applications inside a Docker container running on a remote host, and view the graphics on your local display.

We assume the remote host is an Ubuntu host where:
- Docker has been installed and configured to be used by non-admin users.
- You already have the Astrobee source checked out per `INSTALL.md`.

We'll call the remote host `remotehost` and your local host `laptop`.

On `remotehost`, run these commands to build and run the `astrobee-vnc` container:
```bash
cd $ASTROBEE_WS/src/scripts/docker
./build_vnc.sh
./run_vnc.sh
```

Note that the `run_vnc.sh` command will spout a bunch of warnings that you can safely ignore, and it should keep running indefinitely. If you Ctrl-C out of it you will stop the container.

In another terminal on `remotehost`, run:
```bash
remotehost$ cd $ASTROBEE_WS/src/scripts/docker
remotehost$ ./exec_vnc.sh
docker# xeyes &
```

Note that `exec_vnc.sh` opens up a shell session inside the container. The `xeyes` command demonstrates running an arbitrary X11 application inside the container that will connect to the display of the X server running inside the container.

On `laptop`, run:
```bash
REMOTE_HOST=remotehost.ndc.nasa.gov  # choose your host
ssh -L 127.0.0.1:5900:localhost:5900 $REMOTE_HOST -N
```

This will set up port forwarding between port 5900 of the `laptop` loopback interface and port 5900 of the `remotehost` loopback interface (note: explicitly restricting the forwarding to the loopback interface `127.0.0.1` is important for security). In turn, the Docker daemon will forward port 5900 of the `remotehost` loopback interface to port 5900 of the container where the VNC server is listening. That will allow us to run a VNC client on `laptop` and connect to the VNC server using localhost port 5900 with no fuss.

Note that the `ssh` port forwarding command should keep running indefinitely. If you Ctrl-C out of it you will stop the port forwarding.

In another terminal on `laptop`, run:
```bash
sudo apt-get install xtightvncviewer
vncviewer localhost::5900 -encodings "copyrect tight hextile zlib corre rre raw"
```

The VNC client should show you an X11 desktop running the Xfce lightweight window manager and the `xeyes` example app we ran earlier. Success! This is the end of the demo.

Note that the `-encodings` argument to `vncviewer` tells it to prefer more compressed encodings for low-bandwidth network connections. If you leave it out, `vncviewer` will notice that it is connecting to a VNC server on localhost, decide it doesn't need to worry about bandwidth, and default to an uncompressed `raw` encoding with poor network performance.

## Limitations ##

The `xvfb` server by default will probably not support modern X extensions such as RANDR and GLX that may be used by 3D-heavy applications like Gazebo and the Astrobee GDS Control Station. There are known workarounds for this, but it's currently left for future work. (Potential untested workaround: enable the extensions at the `xvfb` command line, install VirtualGL, and run the apps that need the extensions under `vglrun`.)

The port should be configurable. Hard coding 5900 will cause conflicts if multiple users work on the same remote host. The VNC server is currently not configured to require authentication once the port forwarding is set up, so in theory you could join another user`s graphical session (is that a bug or a feature?).
