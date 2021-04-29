# XMD script for programming the Zedboard FPGA, downloading an ELF image, and
# running the executable.
#
connect arm hw
rst
source ../hw_spec/ps7_init.tcl
ps7_init
# Download the bit file to the FPGA
fpga -f ../hw_spec/system.bit
ps7_post_config
dow Debug/stream_engine_test.elf
run
