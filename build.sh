#!/bin/bash

CONFIRM=$(wget --quiet --save-cookies ./cookies.txt --keep-session-cookies --no-check-certificate "https://drive.google.com/a/kinova.ca/uc?id=1ASbEsulf5cByru8Hy1oBZJyNDBa9H22C&export=download" -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')
wget --load-cookies ./cookies.txt "https://drive.google.com/a/kinova.ca/uc?id=1ASbEsulf5cByru8Hy1oBZJyNDBa9H22C&export=download&confirm=$CONFIRM" -O kortex_api-1.1.6.zip
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while fetching the kortex api. code = ${RESULT}"
    exit $?
fi

rm ./cookies.txt
unzip -d kortex_api kortex_api-1.1.6.zip
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while extracting the kortex api. code = ${RESULT}"
    exit $?
fi

cp -R kortex_api/cpp/linux_gcc_x86-64/include/ src/ros_kortex/kortex_api/
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while copying the kortex api header files. code = ${RESULT}"
    exit $?
fi

cp -R kortex_api/cpp/linux_gcc_x86-64/lib/ src/ros_kortex/kortex_api/lib/
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while copying the kortex api library. code = ${RESULT}"
    exit $?
fi

chmod +x src/ros_kortex/kortex_api/lib/release/libKortexApi.a
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing chmod +x on the kortex api library. code = ${RESULT}"
    exit $?
fi

rm -rf kortex_api/ kortex_api-1.1.6.zip

. /opt/ros/kinetic/setup.bash
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while sourcing /opt/ros/kinetic/setup.bash. code = ${RESULT}"
    exit $?
fi

catkin_make
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing catkin_make. code = ${RESULT}"
    exit $?
fi

exit ${RESULT}
