ALGO = radar_dsp

include $(ROOTDIR)/build/makerules/platform.mk
include $(ROOTDIR)/build/makerules/env.mk

radar_dsp:

ifeq ($(PROC_DSP1_INCLUDE),yes)
	$(MAKE) $(TARGET) CORE=c66xdsp_1
	$(MKDIR) -p $(ROOTDIR)/examples/tda2xx/src/radar_fusion/alg_plugins/radarprocessing/lib
	$(CP) $(DEST_ROOT)/lib/c66xdsp_1/$(PROFILE_c66xdsp_1)/radar_dsp.ae66 $(ROOTDIR)/examples/tda2xx/src/radar_fusion/alg_plugins/radarprocessing/lib/radar_dsp_c66xdsp_1_$(PROFILE_c66xdsp_1).ae66
endif

ifeq ($(PROC_DSP2_INCLUDE),yes)
	$(MAKE) $(TARGET) CORE=c66xdsp_2
	$(MKDIR) -p $(ROOTDIR)/examples/tda2xx/src/radar_fusion/alg_plugins/radarprocessing/lib
	$(CP) $(DEST_ROOT)/lib/c66xdsp_2/$(PROFILE_c66xdsp_2)/radar_dsp.ae66 $(ROOTDIR)/examples/tda2xx/src/radar_fusion/alg_plugins/radarprocessing/lib/radar_dsp_c66xdsp_2_$(PROFILE_c66xdsp_2).ae66
endif

clean:
ifeq ($(PROC_DSP1_INCLUDE),yes)
	$(MAKE) clean CORE=c66xdsp_1
endif
ifeq ($(PROC_DSP2_INCLUDE),yes)
	$(MAKE) clean CORE=c66xdsp_2
endif


all: clean radar_dsp


