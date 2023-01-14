#!/usr/bin/env bash
set -e

# Check that board is properly defined
if [ -z ${ARDUINO_BOARD}] && [ -z $1 ]; then
	echo 'ARDUINO_BOARD variable is not defined.'
	exit 1
fi

if [ -z ${ARDUINO_BOARD} ] && [ -n $1 ]; then
	ARDUINO_BOARD=$1
fi

# Setup build directory
ROOT_DIR=$PWD
BUILD_DIR="$ROOT_DIR/build"
BOARD_DIR="$BUILD_DIR/can-lib_$ARDUINO_BOARD"
rm -rf $BOARD_DIR
rm -f $BUILD_DIR/$ARDUINO_BOARD-can-lib.zip
mkdir -p $BOARD_DIR

# Copy files in correct structure
cp $ROOT_DIR/src/arduino/$ARDUINO_BOARD*.cpp -r $BOARD_DIR
cp $ROOT_DIR/src/*.cpp -r $BOARD_DIR
cp $ROOT_DIR/inc/*.h $BOARD_DIR
cp -r $ROOT_DIR/inc/$ARDUINO_BOARD/*.h $BOARD_DIR
cp -r $ROOT_DIR/inc/$ARDUINO_BOARD/library.properties $BOARD_DIR
cp -r $ROOT_DIR/arduino_lib/* $BOARD_DIR
cp -r $ROOT_DIR/lib/**/inc/* $BOARD_DIR

# Create zip file
cd $BUILD_DIR
zip -r $BUILD_DIR/$ARDUINO_BOARD-can-lib.zip can-lib_$ARDUINO_BOARD/*