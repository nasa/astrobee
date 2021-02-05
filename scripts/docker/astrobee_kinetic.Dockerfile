# This will set up an Astrobee kinetic docker container using the non-NASA install instructions.
# This image builds on top of the base kinetic image building the code.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:base-latest-kinetic

ENV USERNAME astrobee

COPY . /src/astrobee
RUN /src/astrobee/scripts/configure.sh -l -F -D -T -p /opt/astrobee -b /build/astrobee
RUN cd /build/astrobee && make -j`nproc`

COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources
