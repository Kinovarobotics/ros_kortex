#!/bin/bash

wget --no-check-certificate 'https://drive.google.com/a/kinova.ca/uc?authuser=0&id=19zfCNlRUfNBbZoMW9LOpLjVrYOO2BwYb&export=download' -O kortex_api-1.1.3.zip
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while fetching the kortex api. code = ${RESULT}"
    exit $?
fi

unzip kortex_api-1.1.3.zip
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while extracting the kortex api. code = ${RESULT}"
    exit $?
fi

cp -R kortex_api/cpp/linux_x86/include/ src/ros_kortex/kortex_api/
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while copying the kortex api header files. code = ${RESULT}"
    exit $?
fi

rm -r kortex_api/cpp/linux_x86/include/ src/ros_kortex/kortex_api/include/google/
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while deleting unused include folder. code = ${RESULT}"
    exit $?
fi

cp -R kortex_api/cpp/linux_x86/lib/release/ src/ros_kortex/kortex_api/lib/
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while copying the kortex api library. code = ${RESULT}"
    exit $?
fi

chmod +x src/ros_kortex/kortex_api/lib/release/libCppKinovaApi.a
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing chmod +x on the kortex api library. code = ${RESULT}"
    exit $?
fi

. /opt/ros/kinetic/setup.bash
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while sourcing /opt/ros/kinetic/setup.bash. code = ${RESULT}"
    exit $?
fi

catkin_make install --pkg kortex_driver kortex_actuator_driver kortex_device_manager kortex_vision_config_driver
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing catkin_make install. code = ${RESULT}"
    exit $?
fi

catkin_make
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing catkin_make. code = ${RESULT}"
    exit $?
fi

exit ${RESULT}
