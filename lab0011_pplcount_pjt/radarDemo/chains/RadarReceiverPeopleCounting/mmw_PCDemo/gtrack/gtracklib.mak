###################################################################################
# GTRACK Library Makefile
###################################################################################
.PHONY: gTrackLib gTrackLibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src
vpath %.c platform

###################################################################################
# Driver Source Files:
###################################################################################
GTRACK_LIB_SOURCES = gtrackModuleCreate.c		\
					 gtrackModuleDelete.c		\
					 gtrackModuleStep.c			\
					 gtrackModuleConstants.c	\
					 gtrackUnitCreate.c			\
					 gtrackUnitDelete.c			\
					 gtrackUnitEvent.c			\
					 gtrackUnitPredict.c		\
					 gtrackUnitReport.c			\
					 gtrackUnitScore.c			\
					 gtrackUnitStart.c			\
					 gtrackUnitStop.c			\
					 gtrackUnitUpdate.c			\
					 gtrackUtilities.c			\
					 gtrackMatrixMath.c			\
					 gtrackListlib.c 			

###################################################################################
# Driver Source Files:
# - XWR14xx/XWR16xx:
#   GTRACK Library is available for the R4
###################################################################################
ifeq ($(MMWAVE_SDK_DEVICE_TYPE),xwr14xx)
GTRACK_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
GTRACK_C674_DRV_LIB_OBJECTS =
else
GTRACK_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
GTRACK_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))
endif

###################################################################################
# Driver Dependency:
###################################################################################
GTRACK_R4F_DRV_DEPENDS  = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(R4F_DEP_EXT)))
GTRACK_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# Driver Library Names:
###################################################################################
GTRACK_R4F_DRV_LIB  = lib/libgtrack_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
GTRACK_C674_DRV_LIB = lib/libgtrack_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)

R4F_CFLAGS += -i../ -i ../include

###################################################################################
# GTRACK Library Build:
# - XWR14xx: Build the R4 Library
# - XWR16xx: Build the R4 & DSP Library
###################################################################################
gTrackLib: buildDirectories $(GTRACK_R4F_DRV_LIB_OBJECTS) $(GTRACK_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(R4F_AR) $(R4F_AR_OPTS) $(GTRACK_R4F_DRV_LIB) $(GTRACK_R4F_DRV_LIB_OBJECTS)
ifeq ($(MMWAVE_SDK_DEVICE_TYPE),xwr16xx)
	$(C674_AR) $(C674_AR_OPTS) $(GTRACK_C674_DRV_LIB) $(GTRACK_C674_DRV_LIB_OBJECTS)
endif
###################################################################################
# Clean the GTRACK Libraries
###################################################################################
gTrackLibClean:
	@echo 'Cleaning the GTRACK Library Objects'
	@$(DEL) $(GTRACK_R4F_DRV_LIB_OBJECTS) $(GTRACK_R4F_DRV_LIB)
	@$(DEL) $(GTRACK_C674_DRV_LIB_OBJECTS) $(GTRACK_C674_DRV_LIB)
	@$(DEL) $(GTRACK_R4F_DRV_DEPENDS) $(GTRACK_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(GTRACK_R4F_DRV_DEPENDS)
-include $(GTRACK_C674_DRV_DEPENDS)

