#!/bin/bash
NAME=ros_ws # replace by the name of your image
TAG=noetic # the tag of your built image
mkdir -p src
# create a shared volume to store the ros_ws
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/src/" \
    --opt o="bind" \
    "${NAME}_${TAG}"

xhost +
docker run \
    --net=host \
    -it \
    --rm \
    --volume="${NAME}_${TAG}:${PWD}/src/:rw" \
    "${NAME}:${TAG}"