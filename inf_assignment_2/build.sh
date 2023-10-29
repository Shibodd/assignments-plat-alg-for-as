set -e

ASSIGNMENT_FOLDER=$(dirname $0)
BUILD_FOLDER=$ASSIGNMENT_FOLDER/build

cmake -H. -DCMAKE_BUILD_TYPE=Debug -G Ninja -B$BUILD_FOLDER
ninja -C $BUILD_FOLDER