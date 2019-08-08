#!/bin/sh

export PYTHONUNBUFFERED=1
mkdir build
protoc --plugin=protoc-gen-custom=ros_kortex_generator.py -I../protos/ --custom_out=./build ../protos/*.proto
rm -rf build
