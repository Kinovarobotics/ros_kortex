#!/bin/bash

cd kortex_actuator_driver
protoc --plugin=protoc-gen-custom=kortex_actuator_driver.sh -I./protos/ --custom_out=./build ./protos/*.proto

cd ../kortex_device_manager
protoc --plugin=protoc-gen-custom=kortex_device_manager.sh -I./protos/ --custom_out=./build ./protos/*.proto

cd ../kortex_driver
protoc --plugin=protoc-gen-custom=kortex_driver.sh -I./protos/ --custom_out=./build ./protos/*.proto

cd ../kortex_vision_config_driver
protoc --plugin=protoc-gen-custom=kortex_vision_config_driver.sh -I./protos/ --custom_out=./build ./protos/*.proto

exit 0