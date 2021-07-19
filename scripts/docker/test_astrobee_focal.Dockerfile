# This will test an Astrobee focal docker container.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:latest-focal

# Run tests
RUN cd /build/astrobee && make -j`nproc` tests && make -j`nproc` test
