INCDIR += . ./common $(vision_sdk_PATH)/src/utils_common/include

SRCDIR += chains/src \
          modules/detection/src \
          modules/fft/src \
          modules/rangeDopplerFFT/src \
          modules/clustering/dbscan/src \
          modules/utilities \
		  modules/edma/src \
		  modules/DoA/DML/src \
		  modules/DoA/BF/src

SRC_FILES = debugConfig.c \
            radarProcess.c \
            RADARDEMO_clusteringDBscan.c \
			RADARDEMO_clusteringDBscan_priv.c \
            detectionRagDop.c \
            DSPF_sp_fftSPxSP_opt.c \
            gen_twiddle_fft16x16.c \
            gen_twiddle_fft16x32.c \
            gen_twiddle_fft32x32.c \
            gen_twiddle_fftf.c \
            commonParams.c \
            Dopplerfft.c \
            rangefft.c \
            cycle_measure.c \
			iRadarDSPAlgo.c \
            RadarDSP_Run.c \
            radarOsal_malloc.c \
			edma.c \
			RADARDEMO_aoaEstDML.c \
			RADARDEMO_aoaEstDML_priv.c \
			RADARDEMO_aoaEstBF.c \
			RADARDEMO_aoaEstBF_priv.c 
			

SRCS_c66xdsp_1 += $(SRC_FILES)
SRCS_c66xdsp_2 += $(SRC_FILES)
