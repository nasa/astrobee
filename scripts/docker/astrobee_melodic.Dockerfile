# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:base-latest-melodic

RUN /setup/astrobee/install_luajit.sh
COPY . /src/astrobee
RUN /src/astrobee/scripts/configure.sh -l -F -D -T -p /opt/astrobee -b /build/astrobee
RUN cd /build/astrobee && make -j4

COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources
