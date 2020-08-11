rootdir=$(dirname "$(readlink -f "$0")")
cd $rootdir

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -


docker run -it --rm --name astrobee \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY" \
        --user="astrobee" \
        --gpus all \
      astrobee/astrobee \
    /astrobee_init.sh roslaunch astrobee sim.launch dds:=false
