#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Flirry.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Flirry.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/LeptonSDKEmb32PUB/crc16fast.c ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c ../src/LeptonSDKEmb32PUB/LEPTON_VID.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c ../src/system_config/default/framework/system/clk/src/sys_clk_static.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/disp.c ../src/comms.c ../src/flir.c ../src/main.c ../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk/bsp_sys_init.c ../../../../../../microchip/harmony/v1_08_01/framework/driver/i2c/src/dynamic/drv_i2c.c ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/dynamic/drv_spi.c ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/drv_spi_sys_queue_fifo.c ../../../../../../microchip/harmony/v1_08_01/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon.c ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../../microchip/harmony/v1_08_01/framework/system/dma/src/sys_dma.c ../../../../../../microchip/harmony/v1_08_01/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v1_08_01/framework/system/tmr/src/sys_tmr.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/631367192/crc16fast.o ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o ${OBJECTDIR}/_ext/977502197/drv_spi_api.o ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1360937237/disp.o ${OBJECTDIR}/_ext/1360937237/comms.o ${OBJECTDIR}/_ext/1360937237/flir.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o ${OBJECTDIR}/_ext/979991644/drv_i2c.o ${OBJECTDIR}/_ext/130326126/drv_spi.o ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o ${OBJECTDIR}/_ext/513926747/drv_tmr.o ${OBJECTDIR}/_ext/1424859042/sys_devcon.o ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/689718161/sys_dma.o ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o ${OBJECTDIR}/_ext/2020575026/sys_tmr.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/631367192/crc16fast.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d ${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1360937237/disp.o.d ${OBJECTDIR}/_ext/1360937237/comms.o.d ${OBJECTDIR}/_ext/1360937237/flir.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o.d ${OBJECTDIR}/_ext/979991644/drv_i2c.o.d ${OBJECTDIR}/_ext/130326126/drv_spi.o.d ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o.d ${OBJECTDIR}/_ext/513926747/drv_tmr.o.d ${OBJECTDIR}/_ext/1424859042/sys_devcon.o.d ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/689718161/sys_dma.o.d ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o.d ${OBJECTDIR}/_ext/2020575026/sys_tmr.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/631367192/crc16fast.o ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o ${OBJECTDIR}/_ext/977502197/drv_spi_api.o ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1360937237/disp.o ${OBJECTDIR}/_ext/1360937237/comms.o ${OBJECTDIR}/_ext/1360937237/flir.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o ${OBJECTDIR}/_ext/979991644/drv_i2c.o ${OBJECTDIR}/_ext/130326126/drv_spi.o ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o ${OBJECTDIR}/_ext/513926747/drv_tmr.o ${OBJECTDIR}/_ext/1424859042/sys_devcon.o ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/689718161/sys_dma.o ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o ${OBJECTDIR}/_ext/2020575026/sys_tmr.o

# Source Files
SOURCEFILES=../src/LeptonSDKEmb32PUB/crc16fast.c ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c ../src/LeptonSDKEmb32PUB/LEPTON_VID.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c ../src/system_config/default/framework/system/clk/src/sys_clk_static.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/disp.c ../src/comms.c ../src/flir.c ../src/main.c ../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk/bsp_sys_init.c ../../../../../../microchip/harmony/v1_08_01/framework/driver/i2c/src/dynamic/drv_i2c.c ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/dynamic/drv_spi.c ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/drv_spi_sys_queue_fifo.c ../../../../../../microchip/harmony/v1_08_01/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon.c ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../../microchip/harmony/v1_08_01/framework/system/dma/src/sys_dma.c ../../../../../../microchip/harmony/v1_08_01/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v1_08_01/framework/system/tmr/src/sys_tmr.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Flirry.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048EFM144
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1424859042" 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1
	
else
${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1424859042" 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1424859042/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/631367192/crc16fast.o: ../src/LeptonSDKEmb32PUB/crc16fast.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/crc16fast.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/crc16fast.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/crc16fast.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/crc16fast.o.d" -o ${OBJECTDIR}/_ext/631367192/crc16fast.o ../src/LeptonSDKEmb32PUB/crc16fast.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o: ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o: ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o: ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o: ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o: ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_VID.o: ../src/LeptonSDKEmb32PUB/LEPTON_VID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o ../src/LeptonSDKEmb32PUB/LEPTON_VID.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/977502197/drv_spi_api.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_api.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/639803181/sys_clk_static.o: ../src/system_config/default/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ../src/system_config/default/framework/system/clk/src/sys_clk_static.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1360937237/disp.o: ../src/disp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/disp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1360937237/disp.o.d" -o ${OBJECTDIR}/_ext/1360937237/disp.o ../src/disp.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1360937237/comms.o: ../src/comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comms.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/comms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1360937237/comms.o.d" -o ${OBJECTDIR}/_ext/1360937237/comms.o ../src/comms.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1360937237/flir.o: ../src/flir.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/flir.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1360937237/flir.o.d" -o ${OBJECTDIR}/_ext/1360937237/flir.o ../src/flir.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/561783126/bsp_sys_init.o: ../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/561783126" 
	@${RM} ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/561783126/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/561783126/bsp_sys_init.o.d" -o ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o ../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk/bsp_sys_init.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/979991644/drv_i2c.o: ../../../../../../microchip/harmony/v1_08_01/framework/driver/i2c/src/dynamic/drv_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/979991644" 
	@${RM} ${OBJECTDIR}/_ext/979991644/drv_i2c.o.d 
	@${RM} ${OBJECTDIR}/_ext/979991644/drv_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/979991644/drv_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/979991644/drv_i2c.o.d" -o ${OBJECTDIR}/_ext/979991644/drv_i2c.o ../../../../../../microchip/harmony/v1_08_01/framework/driver/i2c/src/dynamic/drv_i2c.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/130326126/drv_spi.o: ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/dynamic/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/130326126" 
	@${RM} ${OBJECTDIR}/_ext/130326126/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/130326126/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/130326126/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/130326126/drv_spi.o.d" -o ${OBJECTDIR}/_ext/130326126/drv_spi.o ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/dynamic/drv_spi.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o: ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/drv_spi_sys_queue_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1220812606" 
	@${RM} ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o.d" -o ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/drv_spi_sys_queue_fifo.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/513926747/drv_tmr.o: ../../../../../../microchip/harmony/v1_08_01/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/513926747" 
	@${RM} ${OBJECTDIR}/_ext/513926747/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/513926747/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/513926747/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/513926747/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/513926747/drv_tmr.o ../../../../../../microchip/harmony/v1_08_01/framework/driver/tmr/src/dynamic/drv_tmr.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1424859042/sys_devcon.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1424859042" 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1424859042/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1424859042/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1424859042/sys_devcon.o ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1424859042" 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_pic32mz.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/689718161/sys_dma.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/dma/src/sys_dma.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/689718161" 
	@${RM} ${OBJECTDIR}/_ext/689718161/sys_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/689718161/sys_dma.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/689718161/sys_dma.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/689718161/sys_dma.o.d" -o ${OBJECTDIR}/_ext/689718161/sys_dma.o ../../../../../../microchip/harmony/v1_08_01/framework/system/dma/src/sys_dma.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/878445320/sys_int_pic32.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/878445320" 
	@${RM} ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/878445320/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/878445320/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o ../../../../../../microchip/harmony/v1_08_01/framework/system/int/src/sys_int_pic32.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/2020575026/sys_tmr.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2020575026" 
	@${RM} ${OBJECTDIR}/_ext/2020575026/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2020575026/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2020575026/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/2020575026/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/2020575026/sys_tmr.o ../../../../../../microchip/harmony/v1_08_01/framework/system/tmr/src/sys_tmr.c    -no-legacy-libc 
	
else
${OBJECTDIR}/_ext/631367192/crc16fast.o: ../src/LeptonSDKEmb32PUB/crc16fast.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/crc16fast.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/crc16fast.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/crc16fast.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/crc16fast.o.d" -o ${OBJECTDIR}/_ext/631367192/crc16fast.o ../src/LeptonSDKEmb32PUB/crc16fast.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o: ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o: ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o: ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o: ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o: ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/631367192/LEPTON_VID.o: ../src/LeptonSDKEmb32PUB/LEPTON_VID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o ../src/LeptonSDKEmb32PUB/LEPTON_VID.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/977502197/drv_spi_api.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_api.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/639803181/sys_clk_static.o: ../src/system_config/default/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ../src/system_config/default/framework/system/clk/src/sys_clk_static.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1360937237/disp.o: ../src/disp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/disp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1360937237/disp.o.d" -o ${OBJECTDIR}/_ext/1360937237/disp.o ../src/disp.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1360937237/comms.o: ../src/comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comms.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/comms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1360937237/comms.o.d" -o ${OBJECTDIR}/_ext/1360937237/comms.o ../src/comms.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1360937237/flir.o: ../src/flir.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/flir.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1360937237/flir.o.d" -o ${OBJECTDIR}/_ext/1360937237/flir.o ../src/flir.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/561783126/bsp_sys_init.o: ../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/561783126" 
	@${RM} ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/561783126/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/561783126/bsp_sys_init.o.d" -o ${OBJECTDIR}/_ext/561783126/bsp_sys_init.o ../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk/bsp_sys_init.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/979991644/drv_i2c.o: ../../../../../../microchip/harmony/v1_08_01/framework/driver/i2c/src/dynamic/drv_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/979991644" 
	@${RM} ${OBJECTDIR}/_ext/979991644/drv_i2c.o.d 
	@${RM} ${OBJECTDIR}/_ext/979991644/drv_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/979991644/drv_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/979991644/drv_i2c.o.d" -o ${OBJECTDIR}/_ext/979991644/drv_i2c.o ../../../../../../microchip/harmony/v1_08_01/framework/driver/i2c/src/dynamic/drv_i2c.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/130326126/drv_spi.o: ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/dynamic/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/130326126" 
	@${RM} ${OBJECTDIR}/_ext/130326126/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/130326126/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/130326126/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/130326126/drv_spi.o.d" -o ${OBJECTDIR}/_ext/130326126/drv_spi.o ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/dynamic/drv_spi.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o: ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/drv_spi_sys_queue_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1220812606" 
	@${RM} ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o.d" -o ${OBJECTDIR}/_ext/1220812606/drv_spi_sys_queue_fifo.o ../../../../../../microchip/harmony/v1_08_01/framework/driver/spi/src/drv_spi_sys_queue_fifo.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/513926747/drv_tmr.o: ../../../../../../microchip/harmony/v1_08_01/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/513926747" 
	@${RM} ${OBJECTDIR}/_ext/513926747/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/513926747/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/513926747/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/513926747/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/513926747/drv_tmr.o ../../../../../../microchip/harmony/v1_08_01/framework/driver/tmr/src/dynamic/drv_tmr.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1424859042/sys_devcon.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1424859042" 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1424859042/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1424859042/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1424859042/sys_devcon.o ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1424859042" 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/1424859042/sys_devcon_pic32mz.o ../../../../../../microchip/harmony/v1_08_01/framework/system/devcon/src/sys_devcon_pic32mz.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/689718161/sys_dma.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/dma/src/sys_dma.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/689718161" 
	@${RM} ${OBJECTDIR}/_ext/689718161/sys_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/689718161/sys_dma.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/689718161/sys_dma.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/689718161/sys_dma.o.d" -o ${OBJECTDIR}/_ext/689718161/sys_dma.o ../../../../../../microchip/harmony/v1_08_01/framework/system/dma/src/sys_dma.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/878445320/sys_int_pic32.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/878445320" 
	@${RM} ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/878445320/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/878445320/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/878445320/sys_int_pic32.o ../../../../../../microchip/harmony/v1_08_01/framework/system/int/src/sys_int_pic32.c    -no-legacy-libc 
	
${OBJECTDIR}/_ext/2020575026/sys_tmr.o: ../../../../../../microchip/harmony/v1_08_01/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2020575026" 
	@${RM} ${OBJECTDIR}/_ext/2020575026/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2020575026/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2020575026/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/LeptonSDKEmb32PUB" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08_01/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08_01/bsp/pic32mz_ef_sk" -MMD -MF "${OBJECTDIR}/_ext/2020575026/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/2020575026/sys_tmr.o ../../../../../../microchip/harmony/v1_08_01/framework/system/tmr/src/sys_tmr.c    -no-legacy-libc 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Flirry.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v1_08_01/bin/framework/peripheral/PIC32MZ2048EFM144_peripherals.a ../../../../../../microchip/harmony/v1_08_01/framework/tcpip/src/crypto/aes_pic32mx.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_REAL_ICE=1 -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -o dist/${CND_CONF}/${IMAGE_TYPE}/Flirry.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../../microchip/harmony/v1_08_01/bin/framework/peripheral/PIC32MZ2048EFM144_peripherals.a ../../../../../../microchip/harmony/v1_08_01/framework/tcpip/src/crypto/aes_pic32mx.a      -no-legacy-libc      -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,--defsym=_min_heap_size=0,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Flirry.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v1_08_01/bin/framework/peripheral/PIC32MZ2048EFM144_peripherals.a ../../../../../../microchip/harmony/v1_08_01/framework/tcpip/src/crypto/aes_pic32mx.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -o dist/${CND_CONF}/${IMAGE_TYPE}/Flirry.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../../microchip/harmony/v1_08_01/bin/framework/peripheral/PIC32MZ2048EFM144_peripherals.a ../../../../../../microchip/harmony/v1_08_01/framework/tcpip/src/crypto/aes_pic32mx.a      -no-legacy-libc  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Flirry.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
