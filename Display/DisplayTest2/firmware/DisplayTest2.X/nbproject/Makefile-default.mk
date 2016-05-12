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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/system_config/default/framework/system/clk/src/sys_clk_static.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/disp.c ../src/main.c ../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2/bsp_sys_init.c ../../../../../../microchip/harmony/v1_07/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c ../../../../../../microchip/harmony/v1_07/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console.c ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console_appio.c ../../../../../../microchip/harmony/v1_07/framework/system/debug/src/sys_debug.c ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon.c ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon_pic32mx.c ../../../../../../microchip/harmony/v1_07/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v1_07/framework/system/ports/src/sys_ports.c ../../../../../../microchip/harmony/v1_07/framework/system/tmr/src/sys_tmr.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/639803181/sys_clk_static.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1360937237/disp.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o ${OBJECTDIR}/_ext/13427052/drv_tmr.o ${OBJECTDIR}/_ext/2123147479/sys_console.o ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o ${OBJECTDIR}/_ext/1387038053/sys_debug.o ${OBJECTDIR}/_ext/2034968709/sys_devcon.o ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o ${OBJECTDIR}/_ext/1576702044/sys_ports.o ${OBJECTDIR}/_ext/248842347/sys_tmr.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1360937237/disp.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o.d ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o.d ${OBJECTDIR}/_ext/13427052/drv_tmr.o.d ${OBJECTDIR}/_ext/2123147479/sys_console.o.d ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o.d ${OBJECTDIR}/_ext/1387038053/sys_debug.o.d ${OBJECTDIR}/_ext/2034968709/sys_devcon.o.d ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o.d ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o.d ${OBJECTDIR}/_ext/1576702044/sys_ports.o.d ${OBJECTDIR}/_ext/248842347/sys_tmr.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/639803181/sys_clk_static.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1360937237/disp.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o ${OBJECTDIR}/_ext/13427052/drv_tmr.o ${OBJECTDIR}/_ext/2123147479/sys_console.o ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o ${OBJECTDIR}/_ext/1387038053/sys_debug.o ${OBJECTDIR}/_ext/2034968709/sys_devcon.o ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o ${OBJECTDIR}/_ext/1576702044/sys_ports.o ${OBJECTDIR}/_ext/248842347/sys_tmr.o

# Source Files
SOURCEFILES=../src/system_config/default/framework/system/clk/src/sys_clk_static.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/disp.c ../src/main.c ../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2/bsp_sys_init.c ../../../../../../microchip/harmony/v1_07/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c ../../../../../../microchip/harmony/v1_07/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console.c ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console_appio.c ../../../../../../microchip/harmony/v1_07/framework/system/debug/src/sys_debug.c ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon.c ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon_pic32mx.c ../../../../../../microchip/harmony/v1_07/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v1_07/framework/system/ports/src/sys_ports.c ../../../../../../microchip/harmony/v1_07/framework/system/tmr/src/sys_tmr.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/639803181/sys_clk_static.o: ../src/system_config/default/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ../src/system_config/default/framework/system/clk/src/sys_clk_static.c     
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c     
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c     
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c     
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c     
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c     
	
${OBJECTDIR}/_ext/1360937237/disp.o: ../src/disp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/disp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1360937237/disp.o.d" -o ${OBJECTDIR}/_ext/1360937237/disp.o ../src/disp.c     
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c     
	
${OBJECTDIR}/_ext/565384264/bsp_sys_init.o: ../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/565384264" 
	@${RM} ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/565384264/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/565384264/bsp_sys_init.o.d" -o ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o ../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2/bsp_sys_init.c     
	
${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o: ../../../../../../microchip/harmony/v1_07/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1161860978" 
	@${RM} ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o.d" -o ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o ../../../../../../microchip/harmony/v1_07/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c     
	
${OBJECTDIR}/_ext/13427052/drv_tmr.o: ../../../../../../microchip/harmony/v1_07/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/13427052" 
	@${RM} ${OBJECTDIR}/_ext/13427052/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/13427052/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/13427052/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/13427052/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/13427052/drv_tmr.o ../../../../../../microchip/harmony/v1_07/framework/driver/tmr/src/dynamic/drv_tmr.c     
	
${OBJECTDIR}/_ext/2123147479/sys_console.o: ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2123147479" 
	@${RM} ${OBJECTDIR}/_ext/2123147479/sys_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/2123147479/sys_console.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2123147479/sys_console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/2123147479/sys_console.o.d" -o ${OBJECTDIR}/_ext/2123147479/sys_console.o ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console.c     
	
${OBJECTDIR}/_ext/2123147479/sys_console_appio.o: ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console_appio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2123147479" 
	@${RM} ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o.d 
	@${RM} ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2123147479/sys_console_appio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/2123147479/sys_console_appio.o.d" -o ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console_appio.c     
	
${OBJECTDIR}/_ext/1387038053/sys_debug.o: ../../../../../../microchip/harmony/v1_07/framework/system/debug/src/sys_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1387038053" 
	@${RM} ${OBJECTDIR}/_ext/1387038053/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/1387038053/sys_debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1387038053/sys_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1387038053/sys_debug.o.d" -o ${OBJECTDIR}/_ext/1387038053/sys_debug.o ../../../../../../microchip/harmony/v1_07/framework/system/debug/src/sys_debug.c     
	
${OBJECTDIR}/_ext/2034968709/sys_devcon.o: ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2034968709" 
	@${RM} ${OBJECTDIR}/_ext/2034968709/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/2034968709/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2034968709/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/2034968709/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/2034968709/sys_devcon.o ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon.c     
	
${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o: ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2034968709" 
	@${RM} ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o.d" -o ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon_pic32mx.c     
	
${OBJECTDIR}/_ext/893287359/sys_int_pic32.o: ../../../../../../microchip/harmony/v1_07/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/893287359" 
	@${RM} ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/893287359/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/893287359/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o ../../../../../../microchip/harmony/v1_07/framework/system/int/src/sys_int_pic32.c     
	
${OBJECTDIR}/_ext/1576702044/sys_ports.o: ../../../../../../microchip/harmony/v1_07/framework/system/ports/src/sys_ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1576702044" 
	@${RM} ${OBJECTDIR}/_ext/1576702044/sys_ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/1576702044/sys_ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1576702044/sys_ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1576702044/sys_ports.o.d" -o ${OBJECTDIR}/_ext/1576702044/sys_ports.o ../../../../../../microchip/harmony/v1_07/framework/system/ports/src/sys_ports.c     
	
${OBJECTDIR}/_ext/248842347/sys_tmr.o: ../../../../../../microchip/harmony/v1_07/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/248842347" 
	@${RM} ${OBJECTDIR}/_ext/248842347/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/248842347/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/248842347/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/248842347/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/248842347/sys_tmr.o ../../../../../../microchip/harmony/v1_07/framework/system/tmr/src/sys_tmr.c     
	
else
${OBJECTDIR}/_ext/639803181/sys_clk_static.o: ../src/system_config/default/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ../src/system_config/default/framework/system/clk/src/sys_clk_static.c     
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c     
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c     
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c     
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c     
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c     
	
${OBJECTDIR}/_ext/1360937237/disp.o: ../src/disp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/disp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1360937237/disp.o.d" -o ${OBJECTDIR}/_ext/1360937237/disp.o ../src/disp.c     
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c     
	
${OBJECTDIR}/_ext/565384264/bsp_sys_init.o: ../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/565384264" 
	@${RM} ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/565384264/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/565384264/bsp_sys_init.o.d" -o ${OBJECTDIR}/_ext/565384264/bsp_sys_init.o ../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2/bsp_sys_init.c     
	
${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o: ../../../../../../microchip/harmony/v1_07/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1161860978" 
	@${RM} ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o.d" -o ${OBJECTDIR}/_ext/1161860978/drv_pmp_dynamic.o ../../../../../../microchip/harmony/v1_07/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c     
	
${OBJECTDIR}/_ext/13427052/drv_tmr.o: ../../../../../../microchip/harmony/v1_07/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/13427052" 
	@${RM} ${OBJECTDIR}/_ext/13427052/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/13427052/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/13427052/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/13427052/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/13427052/drv_tmr.o ../../../../../../microchip/harmony/v1_07/framework/driver/tmr/src/dynamic/drv_tmr.c     
	
${OBJECTDIR}/_ext/2123147479/sys_console.o: ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2123147479" 
	@${RM} ${OBJECTDIR}/_ext/2123147479/sys_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/2123147479/sys_console.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2123147479/sys_console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/2123147479/sys_console.o.d" -o ${OBJECTDIR}/_ext/2123147479/sys_console.o ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console.c     
	
${OBJECTDIR}/_ext/2123147479/sys_console_appio.o: ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console_appio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2123147479" 
	@${RM} ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o.d 
	@${RM} ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2123147479/sys_console_appio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/2123147479/sys_console_appio.o.d" -o ${OBJECTDIR}/_ext/2123147479/sys_console_appio.o ../../../../../../microchip/harmony/v1_07/framework/system/console/src/sys_console_appio.c     
	
${OBJECTDIR}/_ext/1387038053/sys_debug.o: ../../../../../../microchip/harmony/v1_07/framework/system/debug/src/sys_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1387038053" 
	@${RM} ${OBJECTDIR}/_ext/1387038053/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/1387038053/sys_debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1387038053/sys_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1387038053/sys_debug.o.d" -o ${OBJECTDIR}/_ext/1387038053/sys_debug.o ../../../../../../microchip/harmony/v1_07/framework/system/debug/src/sys_debug.c     
	
${OBJECTDIR}/_ext/2034968709/sys_devcon.o: ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2034968709" 
	@${RM} ${OBJECTDIR}/_ext/2034968709/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/2034968709/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2034968709/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/2034968709/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/2034968709/sys_devcon.o ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon.c     
	
${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o: ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2034968709" 
	@${RM} ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o.d" -o ${OBJECTDIR}/_ext/2034968709/sys_devcon_pic32mx.o ../../../../../../microchip/harmony/v1_07/framework/system/devcon/src/sys_devcon_pic32mx.c     
	
${OBJECTDIR}/_ext/893287359/sys_int_pic32.o: ../../../../../../microchip/harmony/v1_07/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/893287359" 
	@${RM} ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/893287359/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/893287359/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/893287359/sys_int_pic32.o ../../../../../../microchip/harmony/v1_07/framework/system/int/src/sys_int_pic32.c     
	
${OBJECTDIR}/_ext/1576702044/sys_ports.o: ../../../../../../microchip/harmony/v1_07/framework/system/ports/src/sys_ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1576702044" 
	@${RM} ${OBJECTDIR}/_ext/1576702044/sys_ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/1576702044/sys_ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1576702044/sys_ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/1576702044/sys_ports.o.d" -o ${OBJECTDIR}/_ext/1576702044/sys_ports.o ../../../../../../microchip/harmony/v1_07/framework/system/ports/src/sys_ports.c     
	
${OBJECTDIR}/_ext/248842347/sys_tmr.o: ../../../../../../microchip/harmony/v1_07/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/248842347" 
	@${RM} ${OBJECTDIR}/_ext/248842347/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/248842347/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/248842347/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -mappio-debug -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_07/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_07/bsp/pic32mx_usb_sk2" -MMD -MF "${OBJECTDIR}/_ext/248842347/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/248842347/sys_tmr.o ../../../../../../microchip/harmony/v1_07/framework/system/tmr/src/sys_tmr.c     
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v1_07/bin/framework/peripheral/PIC32MX795F512L_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../../microchip/harmony/v1_07/bin/framework/peripheral/PIC32MX795F512L_peripherals.a          -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=_min_heap_size=2048,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v1_07/bin/framework/peripheral/PIC32MX795F512L_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../../microchip/harmony/v1_07/bin/framework/peripheral/PIC32MX795F512L_peripherals.a        -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=2048,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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
