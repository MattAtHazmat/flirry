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

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/disp.c ../src/comms.c ../src/flir.c ../src/main.c ../../../../../../microchip/harmony/v1_08/framework/driver/i2c/src/dynamic/drv_i2c.c ../../../../../../microchip/harmony/v1_08/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/dynamic/drv_spi.c ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/drv_spi_sys_queue_fifo.c ../../../../../../microchip/harmony/v1_08/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v1_08/framework/osal/src/osal_freertos.c ../../../../../../microchip/harmony/v1_08/framework/system/debug/src/sys_debug.c ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon.c ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../../microchip/harmony/v1_08/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v1_08/framework/system/tmr/src/sys_tmr.c ../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk/bsp_sys_init.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/list.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/event_groups.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c ../src/system_config/default/framework/system/clk/src/sys_clk_static.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360937237/disp.o ${OBJECTDIR}/_ext/1360937237/comms.o ${OBJECTDIR}/_ext/1360937237/flir.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1478129844/drv_i2c.o ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o ${OBJECTDIR}/_ext/628464326/drv_spi.o ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o ${OBJECTDIR}/_ext/1012064947/drv_tmr.o ${OBJECTDIR}/_ext/895935136/osal_freertos.o ${OBJECTDIR}/_ext/1132301508/sys_debug.o ${OBJECTDIR}/_ext/1566896890/sys_devcon.o ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o ${OBJECTDIR}/_ext/279862154/sys_tmr.o ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o ${OBJECTDIR}/_ext/1968681477/croutine.o ${OBJECTDIR}/_ext/1968681477/list.o ${OBJECTDIR}/_ext/1968681477/queue.o ${OBJECTDIR}/_ext/1968681477/tasks.o ${OBJECTDIR}/_ext/1968681477/timers.o ${OBJECTDIR}/_ext/1968681477/event_groups.o ${OBJECTDIR}/_ext/107526526/heap_2.o ${OBJECTDIR}/_ext/236124815/port.o ${OBJECTDIR}/_ext/236124815/port_asm.o ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o ${OBJECTDIR}/_ext/977502197/drv_spi_api.o ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360937237/disp.o.d ${OBJECTDIR}/_ext/1360937237/comms.o.d ${OBJECTDIR}/_ext/1360937237/flir.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1478129844/drv_i2c.o.d ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o.d ${OBJECTDIR}/_ext/628464326/drv_spi.o.d ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o.d ${OBJECTDIR}/_ext/1012064947/drv_tmr.o.d ${OBJECTDIR}/_ext/895935136/osal_freertos.o.d ${OBJECTDIR}/_ext/1132301508/sys_debug.o.d ${OBJECTDIR}/_ext/1566896890/sys_devcon.o.d ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o.d ${OBJECTDIR}/_ext/279862154/sys_tmr.o.d ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o.d ${OBJECTDIR}/_ext/1968681477/croutine.o.d ${OBJECTDIR}/_ext/1968681477/list.o.d ${OBJECTDIR}/_ext/1968681477/queue.o.d ${OBJECTDIR}/_ext/1968681477/tasks.o.d ${OBJECTDIR}/_ext/1968681477/timers.o.d ${OBJECTDIR}/_ext/1968681477/event_groups.o.d ${OBJECTDIR}/_ext/107526526/heap_2.o.d ${OBJECTDIR}/_ext/236124815/port.o.d ${OBJECTDIR}/_ext/236124815/port_asm.o.d ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d ${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360937237/disp.o ${OBJECTDIR}/_ext/1360937237/comms.o ${OBJECTDIR}/_ext/1360937237/flir.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1478129844/drv_i2c.o ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o ${OBJECTDIR}/_ext/628464326/drv_spi.o ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o ${OBJECTDIR}/_ext/1012064947/drv_tmr.o ${OBJECTDIR}/_ext/895935136/osal_freertos.o ${OBJECTDIR}/_ext/1132301508/sys_debug.o ${OBJECTDIR}/_ext/1566896890/sys_devcon.o ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o ${OBJECTDIR}/_ext/279862154/sys_tmr.o ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o ${OBJECTDIR}/_ext/1968681477/croutine.o ${OBJECTDIR}/_ext/1968681477/list.o ${OBJECTDIR}/_ext/1968681477/queue.o ${OBJECTDIR}/_ext/1968681477/tasks.o ${OBJECTDIR}/_ext/1968681477/timers.o ${OBJECTDIR}/_ext/1968681477/event_groups.o ${OBJECTDIR}/_ext/107526526/heap_2.o ${OBJECTDIR}/_ext/236124815/port.o ${OBJECTDIR}/_ext/236124815/port_asm.o ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o ${OBJECTDIR}/_ext/977502197/drv_spi_api.o ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o

# Source Files
SOURCEFILES=../src/disp.c ../src/comms.c ../src/flir.c ../src/main.c ../../../../../../microchip/harmony/v1_08/framework/driver/i2c/src/dynamic/drv_i2c.c ../../../../../../microchip/harmony/v1_08/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/dynamic/drv_spi.c ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/drv_spi_sys_queue_fifo.c ../../../../../../microchip/harmony/v1_08/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v1_08/framework/osal/src/osal_freertos.c ../../../../../../microchip/harmony/v1_08/framework/system/debug/src/sys_debug.c ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon.c ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../../microchip/harmony/v1_08/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v1_08/framework/system/tmr/src/sys_tmr.c ../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk/bsp_sys_init.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/list.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/event_groups.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c ../src/system_config/default/framework/system/clk/src/sys_clk_static.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c


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
${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o: ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1566896890" 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
	
${OBJECTDIR}/_ext/236124815/port_asm.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/236124815" 
	@${RM} ${OBJECTDIR}/_ext/236124815/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/236124815/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/236124815/port_asm.o.ok ${OBJECTDIR}/_ext/236124815/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/236124815/port_asm.o.d" "${OBJECTDIR}/_ext/236124815/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/236124815/port_asm.o.d"  -o ${OBJECTDIR}/_ext/236124815/port_asm.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/236124815/port_asm.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
	
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
	
else
${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o: ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1566896890" 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1566896890/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2
	
${OBJECTDIR}/_ext/236124815/port_asm.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/236124815" 
	@${RM} ${OBJECTDIR}/_ext/236124815/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/236124815/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/236124815/port_asm.o.ok ${OBJECTDIR}/_ext/236124815/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/236124815/port_asm.o.d" "${OBJECTDIR}/_ext/236124815/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/236124815/port_asm.o.d"  -o ${OBJECTDIR}/_ext/236124815/port_asm.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/236124815/port_asm.o.asm.d",--gdwarf-2
	
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/disp.o: ../src/disp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/disp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/disp.o.d" -o ${OBJECTDIR}/_ext/1360937237/disp.o ../src/disp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/comms.o: ../src/comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comms.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/comms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/comms.o.d" -o ${OBJECTDIR}/_ext/1360937237/comms.o ../src/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/flir.o: ../src/flir.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/flir.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/flir.o.d" -o ${OBJECTDIR}/_ext/1360937237/flir.o ../src/flir.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1478129844/drv_i2c.o: ../../../../../../microchip/harmony/v1_08/framework/driver/i2c/src/dynamic/drv_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1478129844" 
	@${RM} ${OBJECTDIR}/_ext/1478129844/drv_i2c.o.d 
	@${RM} ${OBJECTDIR}/_ext/1478129844/drv_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1478129844/drv_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1478129844/drv_i2c.o.d" -o ${OBJECTDIR}/_ext/1478129844/drv_i2c.o ../../../../../../microchip/harmony/v1_08/framework/driver/i2c/src/dynamic/drv_i2c.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o: ../../../../../../microchip/harmony/v1_08/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/136368979" 
	@${RM} ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o.d 
	@${RM} ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o.d" -o ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o ../../../../../../microchip/harmony/v1_08/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/628464326/drv_spi.o: ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/dynamic/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/628464326" 
	@${RM} ${OBJECTDIR}/_ext/628464326/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/628464326/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/628464326/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/628464326/drv_spi.o.d" -o ${OBJECTDIR}/_ext/628464326/drv_spi.o ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/dynamic/drv_spi.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o: ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/drv_spi_sys_queue_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/519900266" 
	@${RM} ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o.d 
	@${RM} ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o.d" -o ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/drv_spi_sys_queue_fifo.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1012064947/drv_tmr.o: ../../../../../../microchip/harmony/v1_08/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1012064947" 
	@${RM} ${OBJECTDIR}/_ext/1012064947/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1012064947/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1012064947/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1012064947/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/1012064947/drv_tmr.o ../../../../../../microchip/harmony/v1_08/framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/895935136/osal_freertos.o: ../../../../../../microchip/harmony/v1_08/framework/osal/src/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/895935136" 
	@${RM} ${OBJECTDIR}/_ext/895935136/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/895935136/osal_freertos.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/895935136/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/895935136/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/895935136/osal_freertos.o ../../../../../../microchip/harmony/v1_08/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1132301508/sys_debug.o: ../../../../../../microchip/harmony/v1_08/framework/system/debug/src/sys_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1132301508" 
	@${RM} ${OBJECTDIR}/_ext/1132301508/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/1132301508/sys_debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1132301508/sys_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1132301508/sys_debug.o.d" -o ${OBJECTDIR}/_ext/1132301508/sys_debug.o ../../../../../../microchip/harmony/v1_08/framework/system/debug/src/sys_debug.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1566896890/sys_devcon.o: ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1566896890" 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1566896890/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1566896890/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1566896890/sys_devcon.o ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o: ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1566896890" 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/862267552/sys_int_pic32.o: ../../../../../../microchip/harmony/v1_08/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/862267552" 
	@${RM} ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/862267552/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/862267552/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o ../../../../../../microchip/harmony/v1_08/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/279862154/sys_tmr.o: ../../../../../../microchip/harmony/v1_08/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/279862154" 
	@${RM} ${OBJECTDIR}/_ext/279862154/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/279862154/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/279862154/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/279862154/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/279862154/sys_tmr.o ../../../../../../microchip/harmony/v1_08/framework/system/tmr/src/sys_tmr.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/418692434/bsp_sys_init.o: ../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/418692434" 
	@${RM} ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/418692434/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/418692434/bsp_sys_init.o.d" -o ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o ../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk/bsp_sys_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/croutine.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/croutine.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/croutine.o.d" -o ${OBJECTDIR}/_ext/1968681477/croutine.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/list.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/list.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/list.o.d" -o ${OBJECTDIR}/_ext/1968681477/list.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/queue.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/queue.o.d" -o ${OBJECTDIR}/_ext/1968681477/queue.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/tasks.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/tasks.o.d" -o ${OBJECTDIR}/_ext/1968681477/tasks.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/timers.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/timers.o.d" -o ${OBJECTDIR}/_ext/1968681477/timers.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/event_groups.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/event_groups.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/event_groups.o.d" -o ${OBJECTDIR}/_ext/1968681477/event_groups.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/107526526/heap_2.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/107526526" 
	@${RM} ${OBJECTDIR}/_ext/107526526/heap_2.o.d 
	@${RM} ${OBJECTDIR}/_ext/107526526/heap_2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/107526526/heap_2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/107526526/heap_2.o.d" -o ${OBJECTDIR}/_ext/107526526/heap_2.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/236124815/port.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/236124815" 
	@${RM} ${OBJECTDIR}/_ext/236124815/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/236124815/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/236124815/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/236124815/port.o.d" -o ${OBJECTDIR}/_ext/236124815/port.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/977502197/drv_spi_api.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_api.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/639803181/sys_clk_static.o: ../src/system_config/default/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ../src/system_config/default/framework/system/clk/src/sys_clk_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/1360937237/disp.o: ../src/disp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/disp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/disp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/disp.o.d" -o ${OBJECTDIR}/_ext/1360937237/disp.o ../src/disp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/comms.o: ../src/comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comms.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/comms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/comms.o.d" -o ${OBJECTDIR}/_ext/1360937237/comms.o ../src/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/flir.o: ../src/flir.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/flir.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/flir.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/flir.o.d" -o ${OBJECTDIR}/_ext/1360937237/flir.o ../src/flir.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1478129844/drv_i2c.o: ../../../../../../microchip/harmony/v1_08/framework/driver/i2c/src/dynamic/drv_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1478129844" 
	@${RM} ${OBJECTDIR}/_ext/1478129844/drv_i2c.o.d 
	@${RM} ${OBJECTDIR}/_ext/1478129844/drv_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1478129844/drv_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1478129844/drv_i2c.o.d" -o ${OBJECTDIR}/_ext/1478129844/drv_i2c.o ../../../../../../microchip/harmony/v1_08/framework/driver/i2c/src/dynamic/drv_i2c.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o: ../../../../../../microchip/harmony/v1_08/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/136368979" 
	@${RM} ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o.d 
	@${RM} ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o.d" -o ${OBJECTDIR}/_ext/136368979/drv_pmp_dynamic.o ../../../../../../microchip/harmony/v1_08/framework/driver/pmp/src/dynamic/drv_pmp_dynamic.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/628464326/drv_spi.o: ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/dynamic/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/628464326" 
	@${RM} ${OBJECTDIR}/_ext/628464326/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/628464326/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/628464326/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/628464326/drv_spi.o.d" -o ${OBJECTDIR}/_ext/628464326/drv_spi.o ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/dynamic/drv_spi.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o: ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/drv_spi_sys_queue_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/519900266" 
	@${RM} ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o.d 
	@${RM} ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o.d" -o ${OBJECTDIR}/_ext/519900266/drv_spi_sys_queue_fifo.o ../../../../../../microchip/harmony/v1_08/framework/driver/spi/src/drv_spi_sys_queue_fifo.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1012064947/drv_tmr.o: ../../../../../../microchip/harmony/v1_08/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1012064947" 
	@${RM} ${OBJECTDIR}/_ext/1012064947/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1012064947/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1012064947/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1012064947/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/1012064947/drv_tmr.o ../../../../../../microchip/harmony/v1_08/framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/895935136/osal_freertos.o: ../../../../../../microchip/harmony/v1_08/framework/osal/src/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/895935136" 
	@${RM} ${OBJECTDIR}/_ext/895935136/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/895935136/osal_freertos.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/895935136/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/895935136/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/895935136/osal_freertos.o ../../../../../../microchip/harmony/v1_08/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1132301508/sys_debug.o: ../../../../../../microchip/harmony/v1_08/framework/system/debug/src/sys_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1132301508" 
	@${RM} ${OBJECTDIR}/_ext/1132301508/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/1132301508/sys_debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1132301508/sys_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1132301508/sys_debug.o.d" -o ${OBJECTDIR}/_ext/1132301508/sys_debug.o ../../../../../../microchip/harmony/v1_08/framework/system/debug/src/sys_debug.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1566896890/sys_devcon.o: ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1566896890" 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1566896890/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1566896890/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1566896890/sys_devcon.o ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o: ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1566896890" 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/1566896890/sys_devcon_pic32mz.o ../../../../../../microchip/harmony/v1_08/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/862267552/sys_int_pic32.o: ../../../../../../microchip/harmony/v1_08/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/862267552" 
	@${RM} ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/862267552/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/862267552/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/862267552/sys_int_pic32.o ../../../../../../microchip/harmony/v1_08/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/279862154/sys_tmr.o: ../../../../../../microchip/harmony/v1_08/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/279862154" 
	@${RM} ${OBJECTDIR}/_ext/279862154/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/279862154/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/279862154/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/279862154/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/279862154/sys_tmr.o ../../../../../../microchip/harmony/v1_08/framework/system/tmr/src/sys_tmr.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/418692434/bsp_sys_init.o: ../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk/bsp_sys_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/418692434" 
	@${RM} ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/418692434/bsp_sys_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/418692434/bsp_sys_init.o.d" -o ${OBJECTDIR}/_ext/418692434/bsp_sys_init.o ../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk/bsp_sys_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/croutine.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/croutine.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/croutine.o.d" -o ${OBJECTDIR}/_ext/1968681477/croutine.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/list.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/list.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/list.o.d" -o ${OBJECTDIR}/_ext/1968681477/list.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/queue.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/queue.o.d" -o ${OBJECTDIR}/_ext/1968681477/queue.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/tasks.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/tasks.o.d" -o ${OBJECTDIR}/_ext/1968681477/tasks.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/timers.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/timers.o.d" -o ${OBJECTDIR}/_ext/1968681477/timers.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1968681477/event_groups.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1968681477" 
	@${RM} ${OBJECTDIR}/_ext/1968681477/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1968681477/event_groups.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1968681477/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1968681477/event_groups.o.d" -o ${OBJECTDIR}/_ext/1968681477/event_groups.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/107526526/heap_2.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/107526526" 
	@${RM} ${OBJECTDIR}/_ext/107526526/heap_2.o.d 
	@${RM} ${OBJECTDIR}/_ext/107526526/heap_2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/107526526/heap_2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/107526526/heap_2.o.d" -o ${OBJECTDIR}/_ext/107526526/heap_2.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/236124815/port.o: ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/236124815" 
	@${RM} ${OBJECTDIR}/_ext/236124815/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/236124815/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/236124815/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/236124815/port.o.d" -o ${OBJECTDIR}/_ext/236124815/port.o ../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_tasks.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/977502197/drv_spi_api.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_api.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_api.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_api.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o: ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977502197" 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o.d" -o ${OBJECTDIR}/_ext/977502197/drv_spi_master_ebm_tasks.o ../src/system_config/default/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/639803181/sys_clk_static.o: ../src/system_config/default/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_static.o ../src/system_config/default/framework/system/clk/src/sys_clk_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v1_08/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v1_08/bsp/pic32mz_ef_sk" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../../../../../../microchip/harmony/v1_08/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v1_08/bin/framework/peripheral/PIC32MZ2048EFM144_peripherals.a ../../../../../../microchip/harmony/v1_08/framework/tcpip/src/crypto/aes_pic32mx.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../../microchip/harmony/v1_08/bin/framework/peripheral/PIC32MZ2048EFM144_peripherals.a ../../../../../../microchip/harmony/v1_08/framework/tcpip/src/crypto/aes_pic32mx.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)      -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v1_08/bin/framework/peripheral/PIC32MZ2048EFM144_peripherals.a ../../../../../../microchip/harmony/v1_08/framework/tcpip/src/crypto/aes_pic32mx.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/DisplayTest2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../../microchip/harmony/v1_08/bin/framework/peripheral/PIC32MZ2048EFM144_peripherals.a ../../../../../../microchip/harmony/v1_08/framework/tcpip/src/crypto/aes_pic32mx.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
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
