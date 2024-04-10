# This will test an Astrobee docker container.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
ARG REMOTE=astrobee
ARG REVISION=latest-
FROM ${REMOTE}/astrobee:${REVISION}ubuntu${UBUNTU_VERSION}

# Run tests. See also ../run_tests.sh for explanation.
RUN cd /src/astrobee \
	&& catkin build --no-status --force-color --make-args tests \
	&& { catkin build --no-status --force-color --make-args test -j1 || true; } \
	&& { . devel/setup.sh || true; } \
	&& { success="true" && catkin_test_results build || success="false" || true; } \
	&& { [ "$success" = "true" ] || catkin run_tests --no-status --force-color -j1 || true; } \
	&& [ "$success" = "true" ]
