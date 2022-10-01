#!/usr/bin/env bash
set -e

if [ -z ${ARDUINO_BOARD}] && [ -z $1 ]; then
	echo 'ARDUINO_BOARD variable is not defined.'
	exit 1
fi

if [ -z ${ARDUINO_BOARD} ] && [ -n $1 ]; then
	ARDUINO_BOARD=$1
fi

ROOT_DIR=$PWD
BUILD_DIR="$ROOT_DIR/build"
BOARD_DIR="$BUILD_DIR/can-lib"
rm -rf $BOARD_DIR
mkdir -p $BOARD_DIR

cp $ROOT_DIR/src/$ARDUINO_BOARD.cpp -r $BOARD_DIR
cp $ROOT_DIR/inc/*.h $BOARD_DIR
cp -r $ROOT_DIR/inc/$ARDUINO_BOARD $BOARD_DIR
cp -r $ROOT_DIR/arduino_lib/* $BOARD_DIR

rm -f $BUILD_DIR/$ARDUINO_BOARD.zip
cd $BUILD_DIR
zip -r $BUILD_DIR/$ARDUINO_BOARD-can-lib.zip can-lib/*