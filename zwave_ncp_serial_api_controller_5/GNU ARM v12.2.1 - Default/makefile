################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/subdir.mk
-include simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/portable/SiliconLabs/cortexm/subdir.mk
-include simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/portable/MemMang/subdir.mk
-include simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/portable/GCC/ARM_CM33_NTZ/non_secure/subdir.mk
-include simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/subdir.mk
-include simplicity_sdk_2025.6.2/util/third_party/freertos/cmsis/Source/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/components/zw_power_manager/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/components/zw_log/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/AppsHw/src/SerialAPI/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ProtocolConfig/src/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/_commonIF/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/EventHandling/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/AppName/subdir.mk
-include simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/udelay/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/sleeptimer/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/sl_main/src/rtos/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/sl_main/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/power_manager/src/sleep_loop/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/power_manager/src/common/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/mpu/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/memory_manager/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/iostream/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/interrupt_manager/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/hfxo_manager/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/device_manager/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/device_manager/devices/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/device_manager/clocks/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/device_init/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/clock_manager/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/security/sl_component/sli_psec_osal/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/security/sl_component/sl_mbedtls_support/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/radio/rail_lib/plugin/rail_util_power_manager_init/subdir.mk
-include simplicity_sdk_2025.6.2/platform/radio/rail_lib/plugin/pa-conversions/subdir.mk
-include simplicity_sdk_2025.6.2/platform/peripheral/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/emlib/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/emdrv/gpiointerrupt/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/driver/leddrv/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/driver/gpio/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/driver/debug/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/common/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/core/flash/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/app_properties/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/api/subdir.mk
-include simplicity_sdk_2025.6.2/platform/Device/SiliconLabs/EFR32ZG23/Source/subdir.mk
-include simplicity_sdk_2025.6.2/platform/CMSIS/RTOS2/Source/subdir.mk
-include simplicity_sdk_2025.6.2/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_eusart/subdir.mk
-include simplicity_sdk_2025.6.2/hardware/driver/configuration_over_swo/src/subdir.mk
-include simplicity_sdk_2025.6.2/hardware/board/src/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := zwave_ncp_serial_api_controller_5
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: zwave_ncp_serial_api_controller_5.axf

# Tool invocations
zwave_ncp_serial_api_controller_5.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -T "F:\SiLabs_projects\ZWave_NCP_SerialAPI_Controller_Solution\zwave_ncp_serial_api_controller_5\autogen\linkerfile.ld" -Wl,--wrap=_free_r -Wl,--wrap=_malloc_r -Wl,--wrap=_calloc_r -Wl,--wrap=_realloc_r -Wl,--wrap=main -Wl,--start-group -lgcc -lc -Wl,--end-group -flto -Wl,--no-warn-rwx-segments -Xlinker --gc-sections -Xlinker -Map="zwave_ncp_serial_api_controller_5.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o zwave_ncp_serial_api_controller_5.axf -Wl,--start-group "./app.o" "./app_node_info.o" "./app_pm_transition_event.o" "./cmd_get_capabilities.o" "./cmd_handlers.o" "./cmd_handlers_invoker.o" "./cmds_dcdc.o" "./cmds_management.o" "./cmds_power_management.o" "./cmds_rf.o" "./cmds_security.o" "./comm_interface.o" "./main.o" "./nvm_backup_restore.o" "./serialapi_file.o" "./utils.o" "./virtual_slave_node_info.o" "./autogen/sl_board_default_init.o" "./autogen/sl_event_handler.o" "./autogen/sl_iostream_handles.o" "./autogen/sl_simple_led_instances.o" "./simplicity_sdk_2025.6.2/hardware/board/src/sl_board_control_gpio.o" "./simplicity_sdk_2025.6.2/hardware/board/src/sl_board_init.o" "./simplicity_sdk_2025.6.2/hardware/driver/configuration_over_swo/src/sl_cos.o" "./simplicity_sdk_2025.6.2/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_eusart/sl_mx25_flash_shutdown.o" "./simplicity_sdk_2025.6.2/platform/CMSIS/RTOS2/Source/os_systick.o" "./simplicity_sdk_2025.6.2/platform/Device/SiliconLabs/EFR32ZG23/Source/startup_efr32zg23.o" "./simplicity_sdk_2025.6.2/platform/Device/SiliconLabs/EFR32ZG23/Source/system_efr32zg23.o" "./simplicity_sdk_2025.6.2/platform/bootloader/api/btl_interface.o" "./simplicity_sdk_2025.6.2/platform/bootloader/api/btl_interface_storage.o" "./simplicity_sdk_2025.6.2/platform/bootloader/app_properties/app_properties.o" "./simplicity_sdk_2025.6.2/platform/bootloader/core/flash/btl_internal_flash.o" "./simplicity_sdk_2025.6.2/platform/common/src/sl_assert.o" "./simplicity_sdk_2025.6.2/platform/common/src/sl_cmsis_os2_common.o" "./simplicity_sdk_2025.6.2/platform/common/src/sl_core_cortexm.o" "./simplicity_sdk_2025.6.2/platform/common/src/sl_slist.o" "./simplicity_sdk_2025.6.2/platform/common/src/sl_syscalls.o" "./simplicity_sdk_2025.6.2/platform/common/src/sli_cmsis_os2_ext_task_register.o" "./simplicity_sdk_2025.6.2/platform/driver/debug/src/sl_debug_swo.o" "./simplicity_sdk_2025.6.2/platform/driver/gpio/src/sl_gpio.o" "./simplicity_sdk_2025.6.2/platform/driver/leddrv/src/sl_led.o" "./simplicity_sdk_2025.6.2/platform/driver/leddrv/src/sl_simple_led.o" "./simplicity_sdk_2025.6.2/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/nvm3.o" "./simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/nvm3_cache.o" "./simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/nvm3_default_common_linker.o" "./simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/nvm3_hal_flash.o" "./simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/nvm3_lock.o" "./simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/nvm3_object.o" "./simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/nvm3_page.o" "./simplicity_sdk_2025.6.2/platform/emdrv/nvm3/src/nvm3_utils.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_acmp.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_burtc.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_cmu.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_dbg.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_emu.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_eusart.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_gpcrc.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_gpio.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_i2c.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_iadc.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_lcd.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_ldma.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_lesense.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_letimer.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_msc.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_opamp.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_pcnt.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_prs.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_rmu.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_system.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_timer.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_usart.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_vdac.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_wdog.o" "./simplicity_sdk_2025.6.2/platform/peripheral/src/sl_hal_gpio.o" "./simplicity_sdk_2025.6.2/platform/peripheral/src/sl_hal_sysrtc.o" "./simplicity_sdk_2025.6.2/platform/radio/rail_lib/plugin/pa-conversions/pa_conversions_efr32.o" "./simplicity_sdk_2025.6.2/platform/radio/rail_lib/plugin/pa-conversions/pa_curves_efr32.o" "./simplicity_sdk_2025.6.2/platform/radio/rail_lib/plugin/rail_util_power_manager_init/sl_rail_util_power_manager_init.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_attestation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_cipher.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_entropy.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_hash.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_key_derivation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_key_handling.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_signature.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_util.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sli_se_manager_mailbox.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_mbedtls_support/src/sl_mbedtls.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_mbedtls_support/src/sl_psa_crypto.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_mbedtls_support/src/sli_psa_crypto.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_psa_driver_common.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_psa_driver_init.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_psa_trng.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_aead.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_builtin_keys.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_cipher.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_key_derivation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_key_management.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_mac.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_signature.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_opaque_driver_aead.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_opaque_driver_cipher.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_opaque_driver_mac.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_opaque_key_derivation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_driver_aead.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_driver_cipher.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_driver_hash.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_driver_mac.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_key_derivation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_version_dependencies.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sli_psec_osal/src/sli_psec_osal_cmsis_rtos2.o" "./simplicity_sdk_2025.6.2/platform/service/clock_manager/src/sl_clock_manager.o" "./simplicity_sdk_2025.6.2/platform/service/clock_manager/src/sl_clock_manager_hal_s2.o" "./simplicity_sdk_2025.6.2/platform/service/clock_manager/src/sl_clock_manager_init.o" "./simplicity_sdk_2025.6.2/platform/service/clock_manager/src/sl_clock_manager_init_hal_s2.o" "./simplicity_sdk_2025.6.2/platform/service/device_init/src/sl_device_init_dcdc_s2.o" "./simplicity_sdk_2025.6.2/platform/service/device_manager/clocks/sl_device_clock_efr32xg23.o" "./simplicity_sdk_2025.6.2/platform/service/device_manager/devices/sl_device_peripheral_hal_efr32xg23.o" "./simplicity_sdk_2025.6.2/platform/service/device_manager/src/sl_device_clock.o" "./simplicity_sdk_2025.6.2/platform/service/device_manager/src/sl_device_gpio.o" "./simplicity_sdk_2025.6.2/platform/service/device_manager/src/sl_device_peripheral.o" "./simplicity_sdk_2025.6.2/platform/service/hfxo_manager/src/sl_hfxo_manager.o" "./simplicity_sdk_2025.6.2/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.o" "./simplicity_sdk_2025.6.2/platform/service/interrupt_manager/src/sl_interrupt_manager_cortexm.o" "./simplicity_sdk_2025.6.2/platform/service/iostream/src/sl_iostream.o" "./simplicity_sdk_2025.6.2/platform/service/iostream/src/sl_iostream_swo.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_dynamic_reservation.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_pool.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_pool_common.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_region.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_retarget.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sli_memory_manager_common.o" "./simplicity_sdk_2025.6.2/platform/service/mpu/src/sl_mpu_s2.o" "./simplicity_sdk_2025.6.2/platform/service/power_manager/src/common/sl_power_manager_common.o" "./simplicity_sdk_2025.6.2/platform/service/power_manager/src/common/sl_power_manager_em4.o" "./simplicity_sdk_2025.6.2/platform/service/power_manager/src/sleep_loop/sl_power_manager.o" "./simplicity_sdk_2025.6.2/platform/service/power_manager/src/sleep_loop/sl_power_manager_debug.o" "./simplicity_sdk_2025.6.2/platform/service/power_manager/src/sleep_loop/sl_power_manager_hal_s2.o" "./simplicity_sdk_2025.6.2/platform/service/sl_main/src/rtos/main_retarget.o" "./simplicity_sdk_2025.6.2/platform/service/sl_main/src/sl_main_init.o" "./simplicity_sdk_2025.6.2/platform/service/sl_main/src/sl_main_init_memory.o" "./simplicity_sdk_2025.6.2/platform/service/sl_main/src/sl_main_kernel.o" "./simplicity_sdk_2025.6.2/platform/service/sleeptimer/src/sl_sleeptimer.o" "./simplicity_sdk_2025.6.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_burtc.o" "./simplicity_sdk_2025.6.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_sysrtc.o" "./simplicity_sdk_2025.6.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_timer.o" "./simplicity_sdk_2025.6.2/platform/service/udelay/src/sl_udelay.o" "./simplicity_sdk_2025.6.2/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/AppName/ZAF_AppName_weak.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/AppTimer.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/AppTimerDeepSleep.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_PrintAppInfo.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_nvm.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_nvm_app.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_retention_register.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_version.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/EventHandling/zaf_event_distributor.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/EventHandling/zaf_event_distributor_ncp.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ApplicationUtilities/_commonIF/ZAF_Common_interface.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/ZAF/ProtocolConfig/src/zaf_protocol_config.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/AppsHw/src/SerialAPI/SerialAPI_hw.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/IADC.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/board_indicator.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/components/zw_log/zw_log.o" "./simplicity_sdk_2025.6.2/protocol/z-wave/platform/SiliconLabs/components/zw_power_manager/zw_power_manager.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/cmsis/Source/cmsis_os2.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/croutine.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/event_groups.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/list.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/queue.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/stream_buffer.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/tasks.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/timers.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/portable/GCC/ARM_CM33_NTZ/non_secure/mpu_wrappers_v2_asm.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/portable/GCC/ARM_CM33_NTZ/non_secure/port.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/portable/MemMang/heap_3.o" "./simplicity_sdk_2025.6.2/util/third_party/freertos/kernel/portable/SiliconLabs/cortexm/tick_power_manager.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/cipher.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/cipher_wrap.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/constant_time.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/platform.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/platform_util.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_aead.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_cipher.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_client.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_driver_wrappers_no_static.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_ecp.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_ffdh.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_hash.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_mac.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_pake.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_rsa.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_se.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_slot_management.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_storage.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_util.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/threading.o" "C:/Users/mbersalona/SimplicityStudio/SDKs/simplicity_sdk//platform/radio/rail_lib/autogen/librail_release/librail_efr32xg23_gcc_release.a" "C:/Users/mbersalona/SimplicityStudio/SDKs/simplicity_sdk//protocol/z-wave/dist/lib/libZWaveController_series2.a" "C:/Users/mbersalona/SimplicityStudio/SDKs/simplicity_sdk//protocol/z-wave/dist/lib/libzpal_EFR32ZG23.a" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building out file: zwave_ncp_serial_api_controller_5.out'
	arm-none-eabi-objcopy "zwave_ncp_serial_api_controller_5.axf" "zwave_ncp_serial_api_controller_5.out"
	@echo ' '

	@echo 'Building hex file: zwave_ncp_serial_api_controller_5.hex'
	arm-none-eabi-objcopy -O ihex "zwave_ncp_serial_api_controller_5.axf" "zwave_ncp_serial_api_controller_5.hex"
	@echo ' '

	@echo 'Building bin file: zwave_ncp_serial_api_controller_5.bin'
	arm-none-eabi-objcopy -O binary "zwave_ncp_serial_api_controller_5.axf" "zwave_ncp_serial_api_controller_5.bin"
	@echo ' '

	@echo 'Building s37 file: zwave_ncp_serial_api_controller_5.s37'
	arm-none-eabi-objcopy -O srec "zwave_ncp_serial_api_controller_5.axf" "zwave_ncp_serial_api_controller_5.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "zwave_ncp_serial_api_controller_5.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) zwave_ncp_serial_api_controller_5.axf
	-@echo ' '

post-build:
	-"C:\SiliconLabs\SimplicityStudio\v5\developer\adapter_packs\commander\commander.exe" postbuild "F:\SiLabs_projects\ZWave_NCP_SerialAPI_Controller_Solution\zwave_ncp_serial_api_controller_5/zwave_ncp_serial_api_controller_5.slpb" --parameter build_dir:"F:\SiLabs_projects\ZWave_NCP_SerialAPI_Controller_Solution\zwave_ncp_serial_api_controller_5\GNU ARM v12.2.1 - Default" && "C:\SiliconLabs\SimplicityStudio\v5\developer\adapter_packs\commander\commander.exe" postbuild "F:\SiLabs_projects\ZWave_NCP_SerialAPI_Controller_Solution\zwave_ncp_serial_api_controller_5//ZWave_NCP_SerialAPI_Controller_Solution.slpb"
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
