# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:latest-kinetic

# Rebuild the code
COPY . /src/astrobee
RUN /src/astrobee/scripts/configure.sh -l -F -D -p /opt/astrobee -b /build/astrobee
RUN cd /build/astrobee && make install -j`nproc`
COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources

# Run tests
RUN cd /build/astrobee && make test && make run_tests