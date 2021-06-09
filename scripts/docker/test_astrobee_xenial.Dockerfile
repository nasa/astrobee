# This will test an Astrobee xenial docker container.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:latest-xenial

# Run tests

COPY . /src/astrobee
RUN cd /build/astrobee && make -j`nproc` tests && make -j`nproc` test
