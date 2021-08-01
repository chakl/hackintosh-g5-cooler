#!/bin/sh

SRC=./websrv
TARGET=./data

CP_FILES=favicon.ico

[ -d "$TARGET" ] && rm -rf $TARGET 
mkdir $TARGET 

cp $SRC/g5-cooler.html $TARGET/index.html
for f in "$CP_FILES"; do
    if [ -f "$SRC/$f" ]; then
        cp $SRC/$f $TARGET/
    fi
done

gzip $TARGET/*
