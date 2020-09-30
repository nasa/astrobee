# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:latest-melodic

# Run tests
RUN cd /build/astrobee && make -j4 tests && make -j4 test