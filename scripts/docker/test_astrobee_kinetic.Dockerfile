# This will test an Astrobee kinetic docker container.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:latest-kinetic

# Run tests

COPY . /src/astrobee
RUN cd /build/astrobee && make -j`nproc` tests && make -j`nproc` test
