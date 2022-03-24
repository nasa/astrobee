# This will test an Astrobee bionic docker container.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
ARG REMOTE=astrobee
FROM ${REMOTE}/astrobee:latest-ubuntu${UBUNTU_VERSION}

# Run tests. See also ../run_tests.sh for explanation.
RUN cd /src/astrobee \
	&& catkin build --no-status --force-color --make-args tests \
	&& { catkin build --no-status --force-color --make-args test -j1 || true; } \
	&& { . devel/setup.sh || true; } \
	&& { success="true" && catkin_test_results build/${package} || success="false" || true; } \
	&& { [ "$success" = "true" ] || catkin run_tests --no-status --force-color -j1 ${package} || true; } \
	&& [ "$success" = "true" ]
