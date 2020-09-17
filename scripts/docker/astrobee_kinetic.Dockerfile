# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:base-latest-kinetic

COPY . /src/astrobee
RUN cd /src/astrobee git submodule update --init --depth 1 description/media
RUN /src/astrobee/scripts/configure.sh -l -F -D -p /opt/astrobee -b /build/astrobee
RUN cd /build/astrobee && make -j`nproc`
COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources
