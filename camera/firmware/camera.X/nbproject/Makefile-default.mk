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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/camera.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/camera.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/bsp_sys_init.c ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c ../src/LeptonSDKEmb32PUB/crc16fast.c ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c ../src/LeptonSDKEmb32PUB/LEPTON_VID.c ../../../../../microchip/harmony/v1_07_01/framework/driver/i2c/src/dynamic/drv_i2c.c ../../../../../microchip/harmony/v1_07_01/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v1_07_01/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v1_07_01/framework/system/ports/src/sys_ports.c ../../../../../microchip/harmony/v1_07_01/framework/system/tmr/src/sys_tmr.c ../src/global_event.c ../src/flir.c ../src/system_config/default/framework/system/clk/src/sys_clk_static.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/main.c ../src/system_config/default/system_tasks.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o ${OBJECTDIR}/_ext/631367192/crc16fast.o ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o ${OBJECTDIR}/_ext/1534923476/drv_i2c.o ${OBJECTDIR}/_ext/2000988373/drv_tmr.o ${OBJECTDIR}/_ext/1986637198/sys_devcon.o ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o ${OBJECTDIR}/_ext/1578261125/sys_ports.o ${OBJECTDIR}/_ext/2116995074/sys_tmr.o ${OBJECTDIR}/_ext/1360937237/global_event.o ${OBJECTDIR}/_ext/1360937237/flir.o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d ${OBJECTDIR}/_ext/631367192/crc16fast.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d ${OBJECTDIR}/_ext/1534923476/drv_i2c.o.d ${OBJECTDIR}/_ext/2000988373/drv_tmr.o.d ${OBJECTDIR}/_ext/1986637198/sys_devcon.o.d ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o.d ${OBJECTDIR}/_ext/1578261125/sys_ports.o.d ${OBJECTDIR}/_ext/2116995074/sys_tmr.o.d ${OBJECTDIR}/_ext/1360937237/global_event.o.d ${OBJECTDIR}/_ext/1360937237/flir.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o ${OBJECTDIR}/_ext/631367192/crc16fast.o ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o ${OBJECTDIR}/_ext/1534923476/drv_i2c.o ${OBJECTDIR}/_ext/2000988373/drv_tmr.o ${OBJECTDIR}/_ext/1986637198/sys_devcon.o ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o ${OBJECTDIR}/_ext/1578261125/sys_ports.o ${OBJECTDIR}/_ext/2116995074/sys_tmr.o ${OBJECTDIR}/_ext/1360937237/global_event.o ${OBJECTDIR}/_ext/1360937237/flir.o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o

# Source Files
SOURCEFILES=../src/bsp_sys_init.c ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c ../src/LeptonSDKEmb32PUB/crc16fast.c ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c ../src/LeptonSDKEmb32PUB/LEPTON_VID.c ../../../../../microchip/harmony/v1_07_01/framework/driver/i2c/src/dynamic/drv_i2c.c ../../../../../microchip/harmony/v1_07_01/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v1_07_01/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v1_07_01/framework/system/ports/src/sys_ports.c ../../../../../microchip/harmony/v1_07_01/framework/system/tmr/src/sys_tmr.c ../src/global_event.c ../src/flir.c ../src/system_config/default/framework/system/clk/src/sys_clk_static.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/main.c ../src/system_config/default/system_tasks.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/camera.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048EFG100
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o: ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1986637198" 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
	
else
${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o: ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1986637198" 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_cache_pic32mz.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1986637198/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o: ../src/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o.d" -o ${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o ../src/bsp_sys_init.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o: ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o: ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o: ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o: ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c     
	
${OBJECTDIR}/_ext/631367192/crc16fast.o: ../src/LeptonSDKEmb32PUB/crc16fast.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/crc16fast.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/crc16fast.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/crc16fast.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/crc16fast.o.d" -o ${OBJECTDIR}/_ext/631367192/crc16fast.o ../src/LeptonSDKEmb32PUB/crc16fast.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o: ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_VID.o: ../src/LeptonSDKEmb32PUB/LEPTON_VID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o ../src/LeptonSDKEmb32PUB/LEPTON_VID.c     
	
${OBJECTDIR}/_ext/1534923476/drv_i2c.o: ../../../../../microchip/harmony/v1_07_01/framework/driver/i2c/src/dynamic/drv_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1534923476" 
	@${RM} ${OBJECTDIR}/_ext/1534923476/drv_i2c.o.d 
	@${RM} ${OBJECTDIR}/_ext/1534923476/drv_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1534923476/drv_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1534923476/drv_i2c.o.d" -o ${OBJECTDIR}/_ext/1534923476/drv_i2c.o ../../../../../microchip/harmony/v1_07_01/framework/driver/i2c/src/dynamic/drv_i2c.c     
	
${OBJECTDIR}/_ext/2000988373/drv_tmr.o: ../../../../../microchip/harmony/v1_07_01/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2000988373" 
	@${RM} ${OBJECTDIR}/_ext/2000988373/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2000988373/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2000988373/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/2000988373/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/2000988373/drv_tmr.o ../../../../../microchip/harmony/v1_07_01/framework/driver/tmr/src/dynamic/drv_tmr.c     
	
${OBJECTDIR}/_ext/1986637198/sys_devcon.o: ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1986637198" 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1986637198/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1986637198/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1986637198/sys_devcon.o ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon.c     
	
${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1986637198" 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_pic32mz.c     
	
${OBJECTDIR}/_ext/974865368/sys_int_pic32.o: ../../../../../microchip/harmony/v1_07_01/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/974865368" 
	@${RM} ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/974865368/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/974865368/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o ../../../../../microchip/harmony/v1_07_01/framework/system/int/src/sys_int_pic32.c     
	
${OBJECTDIR}/_ext/1578261125/sys_ports.o: ../../../../../microchip/harmony/v1_07_01/framework/system/ports/src/sys_ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1578261125" 
	@${RM} ${OBJECTDIR}/_ext/1578261125/sys_ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/1578261125/sys_ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1578261125/sys_ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1578261125/sys_ports.o.d" -o ${OBJECTDIR}/_ext/1578261125/sys_ports.o ../../../../../microchip/harmony/v1_07_01/framework/system/ports/src/sys_ports.c     
	
${OBJECTDIR}/_ext/2116995074/sys_tmr.o: ../../../../../microchip/harmony/v1_07_01/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2116995074" 
	@${RM} ${OBJECTDIR}/_ext/2116995074/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2116995074/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2116995074/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/2116995074/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/2116995074/sys_tmr.o ../../../../../microchip/harmony/v1_07_01/framework/system/tmr/src/sys_tmr.c     
	
${OBJECTDIR}/_ext/1360937237/global_event.o: ../src/global_event.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/global_event.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/global_event.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/global_event.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1360937237/global_event.o.d" -o ${OBJECTDIR}/_ext/1360937237/global_event.o ../src/global_event.c     
	
${OBJECTDIR}/_ext/1360937237/flir.o: ../src/flir.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/flir.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1360937237/flir.o.d" -o ${OBJECTDIR}/_ext/1360937237/flir.o ../src/flir.c     
	
${OBJECTDIR}/_ext/639803181/sys_clk_static.o: ../src/system_config/default/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ../src/system_config/default/framework/system/clk/src/sys_clk_static.c     
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c     
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c     
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c     
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c     
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c     
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c     
	
else
${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o: ../src/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o.d" -o ${OBJECTDIR}/_ext/1360937237/bsp_sys_init.o ../src/bsp_sys_init.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o: ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Service.o ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Service.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o: ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_AGC.o ../src/LeptonSDKEmb32PUB/LEPTON_AGC.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o: ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_I2C_Protocol.o ../src/LeptonSDKEmb32PUB/LEPTON_I2C_Protocol.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o: ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_SDK.o ../src/LeptonSDKEmb32PUB/LEPTON_SDK.c     
	
${OBJECTDIR}/_ext/631367192/crc16fast.o: ../src/LeptonSDKEmb32PUB/crc16fast.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/crc16fast.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/crc16fast.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/crc16fast.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/crc16fast.o.d" -o ${OBJECTDIR}/_ext/631367192/crc16fast.o ../src/LeptonSDKEmb32PUB/crc16fast.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o: ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_SYS.o ../src/LeptonSDKEmb32PUB/LEPTON_SYS.c     
	
${OBJECTDIR}/_ext/631367192/LEPTON_VID.o: ../src/LeptonSDKEmb32PUB/LEPTON_VID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/631367192" 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d 
	@${RM} ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/631367192/LEPTON_VID.o.d" -o ${OBJECTDIR}/_ext/631367192/LEPTON_VID.o ../src/LeptonSDKEmb32PUB/LEPTON_VID.c     
	
${OBJECTDIR}/_ext/1534923476/drv_i2c.o: ../../../../../microchip/harmony/v1_07_01/framework/driver/i2c/src/dynamic/drv_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1534923476" 
	@${RM} ${OBJECTDIR}/_ext/1534923476/drv_i2c.o.d 
	@${RM} ${OBJECTDIR}/_ext/1534923476/drv_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1534923476/drv_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1534923476/drv_i2c.o.d" -o ${OBJECTDIR}/_ext/1534923476/drv_i2c.o ../../../../../microchip/harmony/v1_07_01/framework/driver/i2c/src/dynamic/drv_i2c.c     
	
${OBJECTDIR}/_ext/2000988373/drv_tmr.o: ../../../../../microchip/harmony/v1_07_01/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2000988373" 
	@${RM} ${OBJECTDIR}/_ext/2000988373/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2000988373/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2000988373/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/2000988373/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/2000988373/drv_tmr.o ../../../../../microchip/harmony/v1_07_01/framework/driver/tmr/src/dynamic/drv_tmr.c     
	
${OBJECTDIR}/_ext/1986637198/sys_devcon.o: ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1986637198" 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1986637198/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1986637198/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1986637198/sys_devcon.o ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon.c     
	
${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1986637198" 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/1986637198/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v1_07_01/framework/system/devcon/src/sys_devcon_pic32mz.c     
	
${OBJECTDIR}/_ext/974865368/sys_int_pic32.o: ../../../../../microchip/harmony/v1_07_01/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/974865368" 
	@${RM} ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/974865368/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/974865368/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/974865368/sys_int_pic32.o ../../../../../microchip/harmony/v1_07_01/framework/system/int/src/sys_int_pic32.c     
	
${OBJECTDIR}/_ext/1578261125/sys_ports.o: ../../../../../microchip/harmony/v1_07_01/framework/system/ports/src/sys_ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1578261125" 
	@${RM} ${OBJECTDIR}/_ext/1578261125/sys_ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/1578261125/sys_ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1578261125/sys_ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1578261125/sys_ports.o.d" -o ${OBJECTDIR}/_ext/1578261125/sys_ports.o ../../../../../microchip/harmony/v1_07_01/framework/system/ports/src/sys_ports.c     
	
${OBJECTDIR}/_ext/2116995074/sys_tmr.o: ../../../../../microchip/harmony/v1_07_01/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2116995074" 
	@${RM} ${OBJECTDIR}/_ext/2116995074/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2116995074/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2116995074/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/2116995074/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/2116995074/sys_tmr.o ../../../../../microchip/harmony/v1_07_01/framework/system/tmr/src/sys_tmr.c     
	
${OBJECTDIR}/_ext/1360937237/global_event.o: ../src/global_event.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/global_event.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/global_event.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/global_event.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1360937237/global_event.o.d" -o ${OBJECTDIR}/_ext/1360937237/global_event.o ../src/global_event.c     
	
${OBJECTDIR}/_ext/1360937237/flir.o: ../src/flir.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/flir.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1360937237/flir.o.d" -o ${OBJECTDIR}/_ext/1360937237/flir.o ../src/flir.c     
	
${OBJECTDIR}/_ext/639803181/sys_clk_static.o: ../src/system_config/default/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ../src/system_config/default/framework/system/clk/src/sys_clk_static.c     
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c     
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c     
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c     
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c     
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c     
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v1_07_01/framework" -I"../src/system_config/default/framework" -I"../src/LeptonSDKEmb32PUB" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c     
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/camera.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../microchip/harmony/v1_07_01/bin/framework/peripheral/PIC32MZ2048EFG100_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/camera.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../microchip/harmony/v1_07_01/bin/framework/peripheral/PIC32MZ2048EFG100_peripherals.a            -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=_min_heap_size=0,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/camera.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../microchip/harmony/v1_07_01/bin/framework/peripheral/PIC32MZ2048EFG100_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/camera.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../microchip/harmony/v1_07_01/bin/framework/peripheral/PIC32MZ2048EFG100_peripherals.a        -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/camera.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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
