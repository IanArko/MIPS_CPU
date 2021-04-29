#!/bin/sh

DIR=lab3_submission

rm -rf $DIR $DIR.tar.gz
mkdir -p $DIR
cp README $DIR
cp -R hw/hdl/verilog/mips $DIR
cp -R sim/tests $DIR
tar -czf $DIR.tar.gz $DIR
rm -rf $DIR
