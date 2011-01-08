SET PATH=C:\Keil\ARM\BIN40\;C:\MentorGraphics\2007PADS\SDD_HOME\common\win32\bin;C:\MentorGraphics\2007PADS\SDD_HOME\common\win32\lib;C:/MentorGraphics/2007PADS/MGC_HOME.ixn\bin;C:/MentorGraphics/2007PADS/MGC_HOME.ixn\lib;C:/MentorGraphics/2007PADS/MGC_HOME.ixn\bin;C:/MentorGraphics/2007PADS/MGC_HOME.ixn\lib;C:\windows\system32;C:\Program Files\Common Files\Thunder Network\KanKan\Codecs;D:\cygwin\bin
SET RVCT40INC=C:\Keil\ARM\RV31\INC
SET RVCT40LIB=C:\Keil\ARM\RV31\LIB
SET CPU_TYPE=STM32F103VE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM3210E-EVAL
SET CPU_CLOCK=0x00000000
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_it.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\main.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_rcc.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_gpio.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_wwdg.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\misc.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_adc.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_bkp.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_can.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_crc.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_dac.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_dbgmcu.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_dma.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_exti.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_flash.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_fsmc.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_i2c.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_iwdg.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_pwr.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_rtc.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_sdio.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_spi.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_tim.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_usart.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\system_stm32f10x.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\core_cm3.__i"
"C:\Keil\ARM\BIN40\ArmAsm" --Via ".\obj\startup_stm32f10x_hd._ia"
"C:\Keil\ARM\BIN40\ArmLink" --Via ".\Obj\STM3210E-EVAL.lnp"
C:\Keil\ARM\BIN40\fromelf.exe ".\Obj\STM3210E-EVAL.axf" --i32combined --output ".\Obj\STM3210E-EVAL.hex"
