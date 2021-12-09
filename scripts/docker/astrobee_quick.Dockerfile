# This will set up an Astrobee melodic docker container using the non-NASA install instructions.
# This image builds on top of the base melodic image building the code.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
ARG ROS_VERSION=kinetic
ARG REMOTE=astrobee
ARG REMOTE_CACHED=${REMOTE}

FROM ${REMOTE_CACHED}/astrobee:latest-ubuntu${UBUNTU_VERSION} AS build_cache
FROM ${REMOTE}/astrobee:base-latest-ubuntu${UBUNTU_VERSION}

COPY --from=build_cache /src/astrobee/build /src/astrobee/build/
COPY --from=build_cache /src/astrobee/devel /src/astrobee/devel/
COPY --from=build_cache /src/astrobee/.catkin_tools /src/astrobee/.catkin_tools/

COPY . /src/astrobee/src/

# reset the timestamps of the files to when they were committed, then cakin will only build new stuff
# https://stackoverflow.com/questions/2458042/restore-a-files-modification-time-in-git
COPY .git .git/
RUN cd /src/astrobee/src/ \
    && for f in $(git ls-tree -r -t --full-name --name-only "HEAD") ; do \
    touch -d $(git log --pretty=format:%cI -1 "HEAD" -- "$f") "$f"; \
    done

RUN . /opt/ros/${ROS_VERSION}/setup.sh \
	&& cd /src/astrobee \
    && catkin clean --setup-files \
    && ./src/scripts/configure.sh -l -F -D -T \
	&& CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/src/astrobee/src/cmake \
	&& catkin build

COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources
