# The firmware uses the Kbuild build system. There are 'Kbuild' files in this
# example that outlays what needs to be built. (check src/Kbuild).
#
# The firmware is configured using options in Kconfig files, the
# values of these end up in the .config file in the firmware directory.
#
# By setting the OOT_CONFIG (it is '$(PWD)/oot-config' by default) environment
# variable you can provide a custom configuration. It is important that you
# enable the app-layer. See app-config in this directory for example.

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := ../..

#
# To include header files from other directories
#

EXTRA_CFLAGS += -I$(PWD)/TinyMPC-ADMM/ext/slap/src
EXTRA_CFLAGS += -I$(PWD)/TinyMPC-ADMM/src
EXTRA_CFLAGS += -DSLAP_FLOAT=float
EXTRA_CFLAGS += -DNDEBUG

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config

# OOT_USES_CXX := 1

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
