project(stm32f427 C CXX ASM)

# definition
set(mcu_def
  -DUSE_HAL_DRIVER
  -DSTM32F427xx
  -DARM_MATH_CM4
  -DARM_MATH_MATRIX_CHECK
  -DARM_MATH_ROUNDING
  -D__FPU_USED=1U
  -D__FPU_PRESENT=1U
)

# include path
set(mcu_include
  Inc
  Drivers/STM32F4xx_HAL_Driver/Inc
  Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
  Drivers/CMSIS/Device/ST/STM32F4xx/Include
  Drivers/CMSIS/Include
  Middlewares/Third_Party/FreeRTOS/Source/include
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
  Middlewares/ST/STM32_USB_Device_Library/Core/Inc
  Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

# sources
file(GLOB_RECURSE src "gcc/*.s" "Src/*.c")

# driver sources
file(GLOB_RECURSE driver_src
  "Drivers/*.c"
  "Middlewares/*.c"
)
file(GLOB_RECURSE except_driver_src
  "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c"
)
list(REMOVE_ITEM driver_src ${except_driver_src})

# link script
file(GLOB_RECURSE flash_ld "*.ld")
set(linker_scrpit ${flash_ld})

# complie options
set(mcu_compile_options
  -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 # hardware floating point
  -ffunction-sections -fdata-sections -fno-common -fmessage-length=0 # mcu compile options
  $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp> # enable assembler files preprocessing
)

# link options
set(mcu_link_options
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
  -mcpu=cortex-m4 -mthumb -mthumb-interwork
  -T ${linker_scrpit}
  --specs=nosys.specs
)

add_definitions(${mcu_def})

foreach(directory ${mcu_include})
  include_directories(${directory})
endforeach(directory)

add_compile_options(${mcu_compile_options})
add_link_options(${mcu_link_options})

# driver library
add_library(${PROJECT_NAME}_driver STATIC ${driver_src})

# send compile option to parent file
set(mcu_def ${mcu_def} PARENT_SCOPE)
set(mcu_include ${mcu_include} PARENT_SCOPE)
set(mcu_src ${src} PARENT_SCOPE)
set(mcu_compile_options ${mcu_compile_options} PARENT_SCOPE)
set(mcu_link_options ${mcu_link_options} PARENT_SCOPE)
