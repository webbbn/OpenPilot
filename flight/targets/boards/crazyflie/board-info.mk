BOARD_TYPE          := 0x04
BOARD_REVISION      := 0x03
BOOTLOADER_VERSION  := 0x04
HW_TYPE             := 0x01

MCU                 := cortex-m3
CHIP                := STM32F103CBT
BOARD               := STM32103CB_CF
MODEL               := MD
MODEL_SUFFIX        := _CF

OPENOCD_JTAG_CONFIG := stlink-v2.cfg
OPENOCD_CONFIG      := stm32f1x.stlink.cfg

# Note: These must match the values in link_$(BOARD)_memory.ld
BL_BANK_BASE        := 0x08000000  # Start of bootloader flash
BL_BANK_SIZE        := 0x00003000  # Should include BD_INFO region
FW_BANK_BASE        := 0x08003000  # Start of firmware flash
FW_BANK_SIZE        := 0x0001D000  # Should include FW_DESC_SIZE
EE_BANK_BASE        := 0x0801A000  # EEPROM storage area
EE_BANK_SIZE        := 0x00006000  # Size of EEPROM storage area

FW_DESC_SIZE        := 0x00000064

OSCILLATOR_FREQ     :=   16000000
