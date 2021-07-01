#!/bin/bash

if [[ $1 == "" ]]; then
    echo "Usage ./build.sh your_registry:tag"
    exit 0
fi
full_path=$1

echo "Build image."
docker build -f Dockerfile -t ${full_path} --network=host .

echo "Done."
