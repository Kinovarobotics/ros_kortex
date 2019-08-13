#!/bin/bash

# Designed to be run from working directory kortex_api/scripts

KORTEX_API_FOLDER_PATH="../"
INCLUDE_FOLDER_PATH="${KORTEX_API_FOLDER_PATH}/include"
LIB_FOLDER_PATH="${KORTEX_API_FOLDER_PATH}/lib"

# Check if include and lib directories exist
if [ ! -d ${INCLUDE_FOLDER_PATH} ]; then
    echo "include does not exist, creating directory..."
    mkdir ${INCLUDE_FOLDER_PATH}
fi
if [ ! -d ${LIB_FOLDER_PATH} ]; then
    echo "lib does not exist, creating directory..."
    mkdir ${LIB_FOLDER_PATH}
fi

# Check if include and lib directories are empty
if [ ! -z "$(ls -A ${INCLUDE_FOLDER_PATH})" ]; then
    echo "include is not empty, exiting..."
    exit 0
fi

if [ ! -z "$(ls -A ${LIB_FOLDER_PATH})" ]; then
    echo "lib is not empty, exiting..."
    exit 0
fi

# Download the API from Google Drive
echo "Downloading the Kortex API from the Web..."
wget -q -O kortex_api.zip https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/API/2.0.0/kortex_api_2.0.0.zip
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while fetching the kortex api. code = ${RESULT}"
    exit $?
fi

# Unzip it
unzip -d kortex_api kortex_api.zip > /dev/null
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while extracting the kortex api. code = ${RESULT}"
    exit $?
fi

# Copy the include folder
cp -R kortex_api/cpp/linux_gcc_x86-64/include/ ${KORTEX_API_FOLDER_PATH}
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while copying the kortex api header files. code = ${RESULT}"
    exit $?
fi

# Copy the lib folder
cp -R kortex_api/cpp/linux_gcc_x86-64/lib/ ${KORTEX_API_FOLDER_PATH}
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while copying the kortex api library. code = ${RESULT}"
    exit $?
fi

# Make the libraries executable
chmod +x ${LIB_FOLDER_PATH}/release/libKortexApi.a
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing chmod +x on the kortex api release library. code = ${RESULT}"
    exit $?
fi

chmod +x ${LIB_FOLDER_PATH}/debug/libKortexApi.a
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing chmod +x on the kortex api debug library. code = ${RESULT}"
    exit $?
fi

# Cleanup
rm -rf kortex_api/ kortex_api.zip

exit ${RESULT}
