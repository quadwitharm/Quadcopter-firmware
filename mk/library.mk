# STM32 Cube

CFLAGS += -DSTM32F429xx

STM32CUBE ?= STM32Cube

SRCDIR += \
		  $(STM32CUBE)/Drivers/STM32F4xx_HAL_Driver/Src \
		  $(STM32CUBE)/Middlewares/Third_Party/FreeRTOS/Source \
		  $(STM32CUBE)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
		  $(STM32CUBE)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \

INCDIR += \
		  $(STM32CUBE)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
		  $(STM32CUBE)/Drivers/CMSIS/Include \
		  $(STM32CUBE)/Drivers/STM32F4xx_HAL_Driver/Inc \
		  $(STM32CUBE)/Drivers/BSP/STM32F429I-Discovery \
		  $(STM32CUBE)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
		  $(STM32CUBE)/Middlewares/Third_Party/FreeRTOS/Source/include \
		  $(STM32CUBE)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \

SRC += \
	   $(STM32CUBE)/Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.c \
	   $(STM32CUBE)/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \

SRC += $(wildcard $(addsuffix /*.c,$(SRCDIR))) \
	  $(wildcard $(addsuffix /*.s,$(SRCDIR)))

DSP_LIB ?= $(STM32CUBE)/Drivers/CMSIS/DSP_Lib

DSP_SRC_DIR += \
			   $(DSP_LIB)/Source/BasicMathFunctions \
			   $(DSP_LIB)/Source/CommonTables \
			   $(DSP_LIB)/Source/ComplexMathFunctions \
			   $(DSP_LIB)/Source/ControllerFunctions \
			   $(DSP_LIB)/Source/FastMathFunctions \
			   $(DSP_LIB)/Source/FilteringFunctions \
			   $(DSP_LIB)/Source/MatrixFunctions \
			   $(DSP_LIB)/Source/StatisticsFunctions \
			   $(DSP_LIB)/Source/SupportFunctions \
			   $(DSP_LIB)/Source/TransformFunctions \

USE_LIB += \
			   BasicMathFunctions \
			   CommonTables \
			   ComplexMathFunctions \
			   ControllerFunctions \
			   FastMathFunctions \
			   FilteringFunctions \
			   MatrixFunctions \
			   StatisticsFunctions \
			   SupportFunctions \
			   TransformFunctions \

LIB_FILE = $(addprefix lib/,$(addsuffix .a, $(USE_LIB)))
