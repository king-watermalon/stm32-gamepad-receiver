##############################################################################
# 2.4G 接收器（手柄 / 鼠标双模式）构建脚本
#
# 用法（在工程根目录执行）：
#   make            编译，产物在 build/ 下：receiver.elf / .hex / .bin
#   make flash      SWD 烧录（需 ST-Link 已连接）
#   make size       只看体积
#   make clean      清掉 build/
#
# 工具链用 STM32CubeIDE 自带那一套（不额外装 arm-none-eabi）。
# 注意：make 在 Windows 下以 cmd.exe 作为 shell，所以 build/ 目录先手工建好，
#       本脚本不依赖 mkdir -p。
##############################################################################

TARGET   = receiver
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
Core/Src/led_rgb.c \
Core/Src/si24r1.c \
Core/Src/usb_comp.c \
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
$(HAL_SRC)/stm32f1xx_hal_spi.c

ASM_SOURCES = Core/Startup/startup_stm32f103c8tx.s

C_DEFS = -DUSE_HAL_DRIVER -DSTM32F103xB $(EXTRA_DEFS)

C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include

MCU      = -mcpu=cortex-m3 -mthumb
# -O2：WS2812 位时序在 -O0 下循环开销太大（单 bit 可能超过 1.25us），必须 -O2
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

# SWD 烧录：mode=UR 表示"复位状态下连接"（程序跑飞或刚上电都能连），freq=4000kHz
flash: $(BUILD)/$(TARGET).bin
	"$(PROG)" -c port=SWD mode=UR freq=4000 -w $(BUILD)/$(TARGET).bin 0x08000000 -v -rst

size: $(BUILD)/$(TARGET).elf
	@$(SIZEBIN) $<

clean:
	-@rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

.PHONY: all clean size flash
