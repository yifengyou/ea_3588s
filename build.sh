#!/bin/bash

set -xe

pushd u-boot-v2017
./ea3588s.sh
popd

pushd kernel-5.10.226
./ea3588s.sh
popd

echo "All done!"

