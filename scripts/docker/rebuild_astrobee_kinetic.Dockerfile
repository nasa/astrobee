# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:latest-kinetic

COPY . /src/astrobee
RUN cd /build/astrobee && make install -j`nproc`
COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources
