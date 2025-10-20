################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include simplicity_sdk_2025.6.2/util/third_party/trusted-firmware-m/platform/ext/target/siliconlabs/hse/subdir.mk
-include simplicity_sdk_2025.6.2/util/third_party/trusted-firmware-m/lib/fih/src/subdir.mk
-include simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/udelay/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/service/memory_manager/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/security/sl_component/sl_mbedtls_support/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/emlib/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/common/src/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/security/sha/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/security/ecc/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/security/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/parser/gbl/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/parser/compression/lzma/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/parser/compression/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/driver/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/debug/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/core/flash/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/core/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/communication/xmodem-uart/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/communication/xmodem-parser/subdir.mk
-include simplicity_sdk_2025.6.2/platform/bootloader/app_version/subdir.mk
-include simplicity_sdk_2025.6.2/platform/Device/SiliconLabs/EFR32ZG23/Source/subdir.mk
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


BUILD_ARTIFACT_NAME := bootloader-uart-xmodem-zwave-otw
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: bootloader-uart-xmodem-zwave-otw.axf

# Tool invocations
bootloader-uart-xmodem-zwave-otw.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "F:\SiLabs_projects\ZWave_NCP_SerialAPI_Controller_Solution\bootloader-uart-xmodem-zwave-otw\autogen\linkerfile.ld" -Wl,--wrap=_free_r -Wl,--wrap=_malloc_r -Wl,--wrap=_calloc_r -Wl,--wrap=_realloc_r -fno-lto -Wl,--no-warn-rwx-segments -Xlinker --gc-sections -Xlinker -Map="bootloader-uart-xmodem-zwave-otw.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o bootloader-uart-xmodem-zwave-otw.axf -Wl,--start-group "./simplicity_sdk_2025.6.2/platform/Device/SiliconLabs/EFR32ZG23/Source/startup_efr32zg23.o" "./simplicity_sdk_2025.6.2/platform/Device/SiliconLabs/EFR32ZG23/Source/system_efr32zg23.o" "./simplicity_sdk_2025.6.2/platform/bootloader/app_version/btl_parser_app_version_check.o" "./simplicity_sdk_2025.6.2/platform/bootloader/communication/xmodem-parser/btl_xmodem.o" "./simplicity_sdk_2025.6.2/platform/bootloader/communication/xmodem-uart/btl_comm_xmodem.o" "./simplicity_sdk_2025.6.2/platform/bootloader/communication/xmodem-uart/btl_comm_xmodem_common.o" "./simplicity_sdk_2025.6.2/platform/bootloader/core/btl_bootload.o" "./simplicity_sdk_2025.6.2/platform/bootloader/core/btl_core.o" "./simplicity_sdk_2025.6.2/platform/bootloader/core/btl_main.o" "./simplicity_sdk_2025.6.2/platform/bootloader/core/btl_parse.o" "./simplicity_sdk_2025.6.2/platform/bootloader/core/btl_reset.o" "./simplicity_sdk_2025.6.2/platform/bootloader/core/flash/btl_internal_flash.o" "./simplicity_sdk_2025.6.2/platform/bootloader/debug/btl_debug.o" "./simplicity_sdk_2025.6.2/platform/bootloader/debug/btl_debug_swo.o" "./simplicity_sdk_2025.6.2/platform/bootloader/driver/btl_driver_delay.o" "./simplicity_sdk_2025.6.2/platform/bootloader/driver/btl_driver_uart.o" "./simplicity_sdk_2025.6.2/platform/bootloader/driver/btl_driver_util.o" "./simplicity_sdk_2025.6.2/platform/bootloader/parser/compression/btl_decompress_lzma.o" "./simplicity_sdk_2025.6.2/platform/bootloader/parser/compression/lzma/LzmaDec.o" "./simplicity_sdk_2025.6.2/platform/bootloader/parser/gbl/btl_gbl_custom_tags.o" "./simplicity_sdk_2025.6.2/platform/bootloader/parser/gbl/btl_gbl_format.o" "./simplicity_sdk_2025.6.2/platform/bootloader/parser/gbl/btl_gbl_parser.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/btl_crc16.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/btl_crc32.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/btl_security_aes.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/btl_security_ecdsa.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/btl_security_sha256.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/btl_security_tokens.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/ecc/ecc.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/sha/btl_sha256.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/sha/crypto_sha.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/sha/cryptoacc_sha.o" "./simplicity_sdk_2025.6.2/platform/bootloader/security/sha/se_sha.o" "./simplicity_sdk_2025.6.2/platform/common/src/sl_assert.o" "./simplicity_sdk_2025.6.2/platform/common/src/sl_core_cortexm.o" "./simplicity_sdk_2025.6.2/platform/common/src/sl_syscalls.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_acmp.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_burtc.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_cmu.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_dbg.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_emu.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_eusart.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_gpcrc.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_gpio.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_i2c.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_iadc.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_lcd.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_ldma.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_lesense.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_letimer.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_msc.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_opamp.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_pcnt.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_prs.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_rmu.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_system.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_timer.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_usart.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_vdac.o" "./simplicity_sdk_2025.6.2/platform/emlib/src/em_wdog.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_attestation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_cipher.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_entropy.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_hash.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_key_derivation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_key_handling.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_signature.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sl_se_manager_util.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/se_manager/src/sli_se_manager_mailbox.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_mbedtls_support/src/se_aes.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_mbedtls_support/src/sl_mbedtls.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_psa_driver_common.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_psa_driver_init.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_aead.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_builtin_keys.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_cipher.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_key_derivation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_key_management.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_mac.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_driver_signature.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_opaque_driver_aead.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_opaque_driver_cipher.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_opaque_driver_mac.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_opaque_key_derivation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_driver_aead.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_driver_cipher.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_driver_hash.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_driver_mac.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_transparent_key_derivation.o" "./simplicity_sdk_2025.6.2/platform/security/sl_component/sl_psa_driver/src/sli_se_version_dependencies.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_dynamic_reservation.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_pool.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_pool_common.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_region.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sl_memory_manager_retarget.o" "./simplicity_sdk_2025.6.2/platform/service/memory_manager/src/sli_memory_manager_common.o" "./simplicity_sdk_2025.6.2/platform/service/udelay/src/sl_udelay.o" "./simplicity_sdk_2025.6.2/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/aes.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/constant_time.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/platform.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/platform_util.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_crypto_client.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/psa_util.o" "./simplicity_sdk_2025.6.2/util/third_party/mbedtls/library/threading.o" "./simplicity_sdk_2025.6.2/util/third_party/trusted-firmware-m/lib/fih/src/fih.o" "./simplicity_sdk_2025.6.2/util/third_party/trusted-firmware-m/platform/ext/target/siliconlabs/hse/sli_se.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building out file: bootloader-uart-xmodem-zwave-otw.out'
	arm-none-eabi-objcopy "bootloader-uart-xmodem-zwave-otw.axf" "bootloader-uart-xmodem-zwave-otw.out"
	@echo ' '

	@echo 'Building hex file: bootloader-uart-xmodem-zwave-otw.hex'
	arm-none-eabi-objcopy -O ihex "bootloader-uart-xmodem-zwave-otw.axf" "bootloader-uart-xmodem-zwave-otw.hex"
	@echo ' '

	@echo 'Building bin file: bootloader-uart-xmodem-zwave-otw.bin'
	arm-none-eabi-objcopy -O binary "bootloader-uart-xmodem-zwave-otw.axf" "bootloader-uart-xmodem-zwave-otw.bin"
	@echo ' '

	@echo 'Building s37 file: bootloader-uart-xmodem-zwave-otw.s37'
	arm-none-eabi-objcopy -O srec "bootloader-uart-xmodem-zwave-otw.axf" "bootloader-uart-xmodem-zwave-otw.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "bootloader-uart-xmodem-zwave-otw.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) bootloader-uart-xmodem-zwave-otw.axf
	-@echo ' '

post-build:
	-"C:\SiliconLabs\SimplicityStudio\v5\developer\adapter_packs\commander\commander.exe" postbuild "F:\SiLabs_projects\ZWave_NCP_SerialAPI_Controller_Solution\bootloader-uart-xmodem-zwave-otw/bootloader-uart-xmodem-zwave-otw.slpb" --parameter build_dir:"F:\SiLabs_projects\ZWave_NCP_SerialAPI_Controller_Solution\bootloader-uart-xmodem-zwave-otw\GNU ARM v12.2.1 - Default"
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
