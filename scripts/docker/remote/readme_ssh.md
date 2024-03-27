\page ssh-docker Remote using SSH

# Astrobee Docker container with graphical applications shared via X11 #

These setup instructions assume you want to run graphical applications inside a Docker container running on a remote host, and view the graphics on your local display.

We assume the remote host is an Ubuntu host where:
- Docker has been installed and configured to be used by non-admin users.
- You already have the Astrobee source checked out per `INSTALL.md`.

We'll call the remote host `remotehost` and your local host `laptop`.

On `remotehost`, run these commands to build and run the `astrobee-vnc` container:
```bash
cd $ASTROBEE_WS/src/scripts/docker/remote
./build_ssh.sh
./run_vnc.sh
```

In your ~/.ssh/config file add:
```
Host docker_*
	User root
	ProxyCommand ssh $(echo %h | sed 's/docker_//') nc 172.17.0.1 9090
```

To ssh to the docker container, use:
```bash
ssh -X docker_${SERVER_NAME}
```

From there you can run graphical applications and the interface should be forwarded to your local computer.