# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:latest-kinetic

# Run tests
RUN cd /build/astrobee && make -j4 tests
RUN /bin/bash -c "source /build/astrobee/devel/setup.bash && rostest choreographer test_zones_nominal.test --text"
RUN cd /build/astrobee && make -j4 test