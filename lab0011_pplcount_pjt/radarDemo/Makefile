
MODULE_NAME = radar_dsp

radar_dsp_PLATFORM_DEPENDENCY = no
radar_dsp_CORE_DEPENDENCY = yes

INCDIR = .

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = bios xdc ipc starterware bsp vision_sdk fc xdais edma mathlib dsplib

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON =
SRCS_tda2xx =
CFLAGS_LOCAL_COMMON = $(vision_sdk_CFLAGS)

MODULE_SRC_BASE_PATH := $(vision_sdk_PATH)/algorithms/radar_dsp

SRCDIR  =

include $(MODULE_SRC_BASE_PATH)/SRC_FILES.MK

# Core/SoC/platform specific source files and CFLAGS
# Example:
#   SRCS_<core/SoC/platform-name> =
#   CFLAGS_LOCAL_<core/SoC/platform-name> =

# Include common make files
include $(ROOTDIR)/build/makerules/common.mk

# OBJs and libraries are built by using rule defined in rules_<target>.mk
#     and need not be explicitly specified here

# Nothing beyond this point
