#!/bin/bash
docker build -t picoha-io-fw .

docker run \
    -it \
    --privileged \
    -v $PWD:/work \
    --rm picoha-io-fw bash

