##################################################################################
# GTRACK makefile
##################################################################################
include ../../common/mmwave_sdk.mak
include ./gtracklib.mak

##################################################################################
# SOC Specific Test Targets
##################################################################################
ifeq ($(MMWAVE_SDK_DEVICE_TYPE),xwr14xx)
include ./test/target/$(MMWAVE_SDK_DEVICE_TYPE)/mssTest.mak
else
include ./test/target/$(MMWAVE_SDK_DEVICE_TYPE)/mssTest.mak
include ./test/target/$(MMWAVE_SDK_DEVICE_TYPE)/dssTest.mak
include ./test/target/$(MMWAVE_SDK_DEVICE_TYPE)/gtrackMssTest.mak
endif

###################################################################################
# Standard Targets which need to be implemented by each mmWave SDK module. This
# plugs into the release scripts.
###################################################################################
.PHONY: all clean lib libClean test testClean gtrackTest gtrackTestClean help

##################################################################################
# Build/Clean the driver
##################################################################################

# This builds the GTRACK Library
lib: gTrackLib

# This cleans the GTRACK Library
libClean: gTrackLibClean

##################################################################################
# Test targets which are device specific
# XWR14xx: Build the MSS Unit Test
# XWR16xx: Build the MSS and DSS Unit Test
##################################################################################
ifeq ($(MMWAVE_SDK_DEVICE_TYPE),xwr14xx)
testClean: 	mssTestClean
test: 		mssTest 
else ifeq ($(MMWAVE_SDK_DEVICE_TYPE),xwr16xx)
testClean:	mssTestClean dssTestClean
test:		mssTest dssTest
gtrackMssTest: gtrackMssTest
gtrackTestClean: gtrackTestClean
endif

# Clean: This cleans all the objects
clean: libClean testClean

# Build everything
all: lib test gtrackMssTest

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the GTRACK '
	@echo 'clean                -> Clean out all the objects'
	@echo 'lib                  -> Build the algorithm Library only'
	@echo 'libClean             -> Clean the algorithm Library only'
	@echo 'test                 -> Builds all the unit tests for the SOC'
	@echo 'testClean            -> Cleans the unit tests for the SOC'
	@echo 'gtrackMssTest        -> Builds the gtracker tests for arm'
	@echo 'gtrackMssTestClean   -> Cleans the gtracker tests for arm'
	@echo '****************************************************************************************'

