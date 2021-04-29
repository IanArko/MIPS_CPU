###############################################################################
#                                                                             #
#             GCC MIPS Cross-compiler for EE180 MIPS (Lab 2)                  #
#                                                                             #
# This Makefile downloads and builds a MIPS cross-compiler C toolchain        #
# based on GCC and Binutils.                                                  #
#                                                                             #
# Typical Usage:                                                              #
#   make          : Download and build everything (binutils + gcc).           #
#   make download : Download packages.                                        #
#   make binutils : Download and build binutils.                              #
#   make gcc      : Download and build binutils and gcc.                      #
#   make clean    : Delete the build directory.                               #
#   make cleanall : Delete the build and install directories.                 #
#                                                                             #
# Requirements:                                                               #
#   - GNU Make                                                                #
#   - wget                                                                    #
#   - common utilities (grep, awk)                                            #
#                                                                             #
###############################################################################


#---------- Basic settings ----------#
PKG_LIST  = packages.lst
BLD_DIR   = build
INST_DIR  = mips_tc


#---------- No need to modify below ----------#
PKGS          := $(shell awk '{print $$1;}' < $(PKG_LIST))
PKGS_URL      := $(shell awk '{print $$2;}' < $(PKG_LIST))
PKGS_TBALL    := $(addprefix $(BLD_DIR)/,$(notdir $(PKGS_URL)))
BINUTILS_NAME := $(filter binutils%,$(PKGS))
GCC_NAME      := $(filter gcc%,$(PKGS))
GMP_NAME      := $(filter gmp%,$(PKGS))
MPC_NAME      := $(filter mpc%,$(PKGS))
MPFR_NAME     := $(filter mpfr%,$(PKGS))
NEWLIB_NAME   := $(filter newlib%,$(PKGS))

.PHONY: all download binutils gcc clean cleanall

all: gcc


$(BLD_DIR):
	@mkdir -p $@


download: $(PKGS_TBALL)


$(PKGS_TBALL): $(PKG_LIST) | $(BLD_DIR)
	@echo [download]  $@
	@cd $(BLD_DIR) && wget --quiet --no-use-server-timestamps $(shell grep '$(notdir $@)' < $(PKG_LIST) | awk '{print $$2;}')
	@tar xf $@ -C $(BLD_DIR)


binutils: $(INST_DIR)/bin/mips-elf-as


$(INST_DIR)/bin/mips-elf-as: $(filter $(BLD_DIR)/binutils%,$(PKGS_TBALL)) | $(BLD_DIR)
	@rm -rf $(BLD_DIR)/binutils-build
	@mkdir $(BLD_DIR)/binutils-build
	@cd $(BLD_DIR)/binutils-build && ../$(BINUTILS_NAME)/configure --prefix=$(abspath $(INST_DIR)) \
     --target=mips-elf --disable-nls
	@$(MAKE) -C $(BLD_DIR)/binutils-build
	@$(MAKE) -C $(BLD_DIR)/binutils-build install


gcc: $(INST_DIR)/bin/mips-elf-gcc


$(INST_DIR)/bin/mips-elf-gcc: $(filter $(BLD_DIR)/gcc% $(BLD_DIR)/gmp% $(BLD_DIR)/mpc% $(BLD_DIR)/mpfr% $(BLD_DIR)/newlib%,$(PKGS_TBALL)) | binutils $(BLD_DIR)
	@rm -rf $(BLD_DIR)/gcc-build
	@mkdir $(BLD_DIR)/gcc-build
	@ln -s ../$(GMP_NAME) $(BLD_DIR)/$(GCC_NAME)/gmp
	@ln -s ../$(MPC_NAME) $(BLD_DIR)/$(GCC_NAME)/mpc
	@ln -s ../$(MPFR_NAME) $(BLD_DIR)/$(GCC_NAME)/mpfr
	@ln -s ../$(NEWLIB_NAME)/newlib $(BLD_DIR)/$(GCC_NAME)/newlib
	@ln -s ../$(NEWLIB_NAME)/libgloss $(BLD_DIR)/$(GCC_NAME)/libgloss
	@cd $(BLD_DIR)/gcc-build && ../$(GCC_NAME)/configure --prefix=$(abspath $(INST_DIR)) --target=mips-elf \
     --with-newlib --without-headers --with-gnu-ld --with-gnu-as --disable-libssp --disable-nls --enable-c99 \
     --enable-long-long --enable-languages=c
	@$(MAKE) -C $(BLD_DIR)/gcc-build
	@$(MAKE) -C $(BLD_DIR)/gcc-build install


clean:
	@rm -rf $(BLD_DIR)

cleanall: clean
	@rm -rf $(INST_DIR)
