# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:cross

# Copy astrobee code
COPY . /src/astrobee/src

# Cross-compile
RUN cd /src/astrobee/ && catkin build
