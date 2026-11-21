##############################################################################
# 遥控器 v2（新板）构建脚本
#
# 用法（在工程根目录执行）：
#   make            编译，产物在 build/ 下：controller.elf / .hex / .bin
#   make flash      SWD 烧录
#   make size       只看体积
#   make clean      清掉 build/
#
# 工具链用 STM32CubeIDE 自带那一套，路径与 receiver/ 保持一致。
# 注意：make 在 Windows 下以 cmd.exe 作为 shell，所以 build/ 目录先手工建好。
##############################################################################

TARGET   = controller
BUILD    = build

IDE      = D:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins
TOOL     = $(IDE)/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin
PROG     = $(IDE)/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI.exe

CC       = $(TOOL)/arm-none-eabi-gcc.exe
OBJCOPY  = $(TOOL)/arm-none-eabi-objcopy.exe
SIZEBIN  = $(TOOL)/arm-none-eabi-size.exe

LDSCRIPT = STM32F103C8TX_FLASH.ld
HAL_SRC  = Drivers/STM32F1xx_HAL_Driver/Src

C_SOURCES = \
Core/Src/main.c \
Core/Src/keypad.c \
Core/Src/joystick.c \
Core/Src/battery.c \
Core/Src/led_rgb.c \
Core/Src/si24r1.c \
Core/Src/usb_comp.c \
Core/Src/soft_i2c.c \
Core/Src/oled.c \
Core/Src/oledfont.c \
Core/Src/cjkfont.c \
Core/Src/ui.c \
Core/Src/uart_dbg.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Core/Src/system_stm32f1xx.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
$(HAL_SRC)/stm32f1xx_hal.c \
$(HAL_SRC)/stm32f1xx_hal_cortex.c \
$(HAL_SRC)/stm32f1xx_hal_gpio.c \
$(HAL_SRC)/stm32f1xx_hal_rcc.c \
$(HAL_SRC)/stm32f1xx_hal_rcc_ex.c \
$(HAL_SRC)/stm32f1xx_hal_flash.c \
$(HAL_SRC)/stm32f1xx_hal_flash_ex.c \
$(HAL_SRC)/stm32f1xx_hal_spi.c \
$(HAL_SRC)/stm32f1xx_hal_uart.c \
$(HAL_SRC)/stm32f1xx_hal_adc.c \
$(HAL_SRC)/stm32f1xx_hal_adc_ex.c

ASM_SOURCES = Core/Startup/startup_stm32f103c8tx.s

C_DEFS = -DUSE_HAL_DRIVER -DSTM32F103xB $(EXTRA_DEFS)

C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include

MCU      = -mcpu=cortex-m3 -mthumb
OPT      = -O2 -g3
WARN     = -Wall -Wextra -Wno-unused-parameter
CFLAGS   = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) $(WARN) -ffunction-sections -fdata-sections
ASFLAGS  = $(MCU) $(OPT) -ffunction-sections -fdata-sections
LDFLAGS  = $(MCU) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--print-memory-usage

OBJECTS  = $(addprefix $(BUILD)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c Makefile | $(BUILD)
	@echo "  CC  $<"
	@$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" -o $@ $<

$(BUILD)/%.o: %.s Makefile | $(BUILD)
	@echo "  AS  $<"
	@$(CC) -x assembler-with-cpp -c $(ASFLAGS) -o $@ $<

$(BUILD)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "  LD  $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -lm -o $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	@$(OBJCOPY) -O ihex $< $@
	@echo "  HEX $@"

$(BUILD)/%.bin: $(BUILD)/%.elf
	@$(OBJCOPY) -O binary -S $< $@
	@echo "  BIN $@"

$(BUILD):
	@mkdir $(BUILD)

# mode=normal：不复位连接。mode=UR 在没引出 NRST 的板上会失败。
flash: $(BUILD)/$(TARGET).bin
	"$(PROG)" -c port=SWD mode=normal freq=4000 -w $(BUILD)/$(TARGET).bin 0x08000000 -v -rst

size: $(BUILD)/$(TARGET).elf
	@$(SIZEBIN) $<

clean:
	-@rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

.PHONY: all clean size flash
