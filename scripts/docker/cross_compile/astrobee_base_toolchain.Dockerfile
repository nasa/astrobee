# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM ubuntu:16.04

# Copy over the toolchain
COPY . /arm_cross/toolchain/gcc
