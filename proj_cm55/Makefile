################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Makefile for configuring the Cortex-M55 core project
#
################################################################################
# \copyright
# Copyright 2025, Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

include ../common.mk

################################################################################
# Basic Configuration
################################################################################


# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=proj_cm55
APPTYPE=flash

# If set to "true" or "1", display full command-lines when building.
VERBOSE=


################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS += FREERTOS RTOS_AWARE  


# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS+=

CORE=CM55
CORE_NAME=CM55_0
NPU=U55

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES+= 

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES+=

# Add additional defines to the build process (without a leading -D).
DEFINES += CY_RTOS_AWARE

DEFINES += FLASH_BOOT

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT = hardfp

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ifeq ($(TOOLCHAIN),LLVM_ARM)
    ifeq ($(CONFIG),Debug)
        CFLAGS+=-O1
    else
        CFLAGS+=
    endif
else
    CFLAGS+=
endif 

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS+=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS+=

# Additional / custom linker flags.
LDFLAGS+=

# Additional / custom libraries to link in to the application.
LDLIBS+=

# Path to the linker script to use (if empty, use the default linker script).
 LINKER_SCRIPT=
# Custom pre-build commands to run.
PREBUILD+=

# Custom post-build commands to run.
POSTBUILD+=

################################################################################
# Application specific Configurations to use COMPONENTS, DEFINES and flags.
################################################################################
DEFINES += CY_RETARGET_IO_CONVERT_LF_TO_CRLF 



ifeq ($(CONFIG_USECASE),MUSIC_PLAYER)

# Variable to enable/disable MP3 playback
    ENABLE_MP3_PLAYBACK=0

    COMPONENTS+=MUSICPLAYER 
    DEEPCRAFT_PROJECT_NAME=localvoice_music_nonum
    #Audio playback
    ENABLE_PLAYBACK=Y
    ifeq ($(ENABLE_PLAYBACK),Y)
        DEFINES += AUDIO_OUT
        COMPONENTS += AUDIO_PLAYBACK
        COMPONENTS += MUSIC_PLAYER
    endif
    
    ifeq ($(ENABLE_MP3_PLAYBACK),1)
        COMPONENTS+=MP3_CODEC
        DEFINES+=SUPPORT_MP3_PARSING ENABLE_MP3_DECODE
        DEFINES+=ENABLE_MP3_PLAYBACK
    endif    
    
endif

ifeq ($(CONFIG_GFX),GFX_ENABLE)
    COMPONENTS += MPUI
    COMPONENTS += GFX_UI
    DEFINES += _BAREMETAL=0
    ifeq ($(CONFIG_DISPLAY), W4P3INCH_DISP)
        DEFINES += MTB_DISPLAY_W4P3INCH_RPI MTB_CTP_FT5406
        CY_IGNORE += $(SEARCH_display-tft-ek79007ad3) \$(SEARCH_display-dsi-waveshare-7-0-lcd-c)
    endif
    DEFINES += ENABLE_GFX_UI
endif

#For Profiler
COMPONENTS += PROFILER
DEFINES+=PROFILER_ENABLE

#AFE configuration and related macros
COMPONENTS += CMSIS_DSP

DEFINES += RUN_FLOAT_FE
DEFINES += ENABLE_AFE_MW_SUPPORT
#DEFINES +=AFE_AUDIO_PROCESSING_TASK_PRIORITY=2
COMPONENTS+=MW_DSNS_MODEL MW_MDL_DSNS MW_MDL_DSES
DEFINES+=USE_MTB_ML
DEFINES+=DSNS_ENABLE_MTB_ML
#AFE Tuning enablement
DEFINES += CY_AFE_ENABLE_TUNING_FEATURE

ifeq ($(CONFIG_VOICE_CORE_MODE),LIMITED)
    COMPONENTS+=AVC_DEMO
endif

ifeq ($(CONFIG_VOICE_CORE_MODE),FULL)
    COMPONENTS+=AVC_FULL
endif

#Application logger
COMPONENTS += APP_LOGGER

#Voice Assistant
ifeq ($(CONFIG_INFERENCING_LIB),VOICE_ASSISTANT)
    VA_LIB=Y
    DEFINES+=USE_MTB_ML
    DEFINES += VA_INFERENCING
    COMPONENTS += VOICE_ASSISTANT

    DEFINES += EMBEDDED_DEV    #EMBEDDED_DEV is must to build #IFX algorithms for SOD, LPWWD MW
    DEFINES += ENABLE_IFX_SOD  #ENABLE_IFX_SOD is must to build #IFX algorithms for SOD
    DEFINES += ENABLE_IFX_PRE_PROCESS_HPF #To enable HPF

    DEFINES += ENABLE_IFX_VA_WWD
    DEFINES += ENABLE_IFX_VA_CMD
    DEFINES+=PROJECT_PREFIX=$(DEEPCRAFT_PROJECT_NAME)
    DEFINES +=VA_VARIABLE_ENABLE

endif

# Neural Network Configuration. Options include
#
# float    -- floating point for the input data and weights
# int8x8   -- 8-bit fixed-point for the input data and weights
NN_TYPE=int16x8

# Choose the inference engine
# tflm      -- TensorFlow Lite for Microcontrollers inference engine with interpreter
# Note tflm_less option is not supported on CM55+U55
NN_INFERENCE_ENGINE=tflm

NPU=U55

DEFINES+=ETHOSU55

# Add additional components related to MTB ML
COMPONENTS+=CMSIS_DSP U55

#Default Cache management is all layers.
#DEFINES+=MTB_ML_ETHOSU_CACHE_MGMT_TYPE=MTB_ML_ETHOSU_CACHE_MGMT_ALL_LAYERS


# Add additional define/components to select the inference engine
ifeq (tflm, $(NN_INFERENCE_ENGINE))
    COMPONENTS+=ML_TFLM
    DEFINES+=TF_LITE_STATIC_MEMORY
endif

DEFINES+= CY_ML_MODEL_MEM=.cy_socmem_data
ifeq (int16x8, $(NN_TYPE))
    COMPONENTS+=ML_INT16x8
endif
ifeq (int8x8, $(NN_TYPE))
    COMPONENTS+=ML_INT8x8
endif

#For USB building
COMPONENTS += EMUSB
COMPONENTS += USBD_BASE
       
#TLV Codec related settings
#Choose between Headphone and on-board speaker
#USE_HEADPHONE/USE_SPEAKER
DEFINES+=USE_SPEAKER
#Set processing block to 0x01 for 48KHz playback
DEFINES+=TLV320DAC3100_PROCESSING_BLOCK=0x01

#PDM mic settings - EVK 
#GAIN_CONTROL_ON - to restrict gain level to -25.0dB to 25.0dB
DEFINES +=GAIN_CONTROL_ON


################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field 
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by 
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level 
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../..

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# software provided compiler by default).
CY_COMPILER_PATH?=


CY_AVC_PATH=$(SEARCH_audio-voice-core)
CY_AFE_PATH=$(SEARCH_audio-front-end)
CY_SOD_PATH=$(SEARCH_speech-onset-detection)

# Ignore unused LVGL middleware files from compilation

#LVGL_PATH = ./source/modules/use_case/COMPONENT_MUSICPLAYER/COMPONENT_LVGL
ifeq ($(CONFIG_GFX),GFX_ENABLE)
    CY_IGNORE += $(SEARCH_lvgl)/src/others/vg_lite_tvg $(SEARCH_lvgl)/src/libs/thorvg $(SEARCH_lvgl)/tests $(SEARCH_lvgl)/draw/nxp $(SEARCH_lvgl)/draw/renesas $(SEARCH_lvgl)/examples $(SEARCH_lvgl)/src/draw/vg_lite/lv_draw_vg_lite.c $(SEARCH_lvgl)/src/draw/vg_lite/lv_vg_lite_utils.c $(SEARCH_lvgl)/src/draw/vg_lite/lv_draw_vg_lite_img.c
    CY_IGNORE += $(SEARCH_lvgl)/src/draw/sw/blend/helium $(SEARCH_lvgl)/src/draw/sw/blend/neon $(SEARCH_lvgl)/demos/music $(SEARCH_lvgl)/src/core/lv_refr.c#TODO ignore 
endif

ifeq ($(CONFIG_GFX),GFX_DISABLE)
    CY_IGNORE += $(SEARCH_lvgl)
endif


include $(CY_TOOLS_DIR)/make/start.mk
