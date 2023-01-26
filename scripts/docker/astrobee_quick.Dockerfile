# This will set up an Astrobee melodic docker container using the non-NASA
# install instructions. This image builds on top of the base melodic image
# building the code. You must set the docker context to be the repository root
# directory
#
# This dockerfile is an alternative to `astrobee.Dockerfile`. It uses previous
# docker image build artifacts to speed up the catkin build step. If, in the
# future, you were to use a 'quick' astrobee's image as a build cache, be sure
# to add `catkin clean --orphans` so you don't bloat the cache image down the
# line.

ARG UBUNTU_VERSION=16.04
ARG REMOTE=astrobee
ARG REMOTE_CACHED=${REMOTE}

FROM ${REMOTE_CACHED}/astrobee:latest-ubuntu${UBUNTU_VERSION} AS build_cache
FROM ${REMOTE}/astrobee:latest-base-ubuntu${UBUNTU_VERSION}

ARG ROS_VERSION=kinetic

COPY --from=build_cache /src/astrobee/build /src/astrobee/build/
COPY --from=build_cache /src/astrobee/devel /src/astrobee/devel/
COPY --from=build_cache /src/astrobee/.catkin_tools /src/astrobee/.catkin_tools/

COPY . /src/astrobee/src/

# reset the timestamps of the files to when they were committed, then cakin will only build new stuff
# https://stackoverflow.com/questions/2458042/restore-a-files-modification-time-in-git
RUN cd /src/astrobee/src/ \
    && \
    for f in $(git ls-tree -r -t --full-name --name-only "HEAD") ; do \
        touch -m -d $(git log --pretty=format:%cI -1 "HEAD" -- "$f") "$f"; \
    done

RUN . /opt/ros/${ROS_VERSION}/setup.sh \
	&& cd /src/astrobee \
    && catkin clean --setup-files \
    && ./src/scripts/configure.sh -l -F -D -T \
	&& CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/src/astrobee/src/cmake \
	&& catkin build --status-rate 0.01

COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources
