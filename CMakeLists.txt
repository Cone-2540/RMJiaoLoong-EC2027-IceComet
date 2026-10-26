set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.16)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# c/c++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "-std=c99")

# optimize level
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(STATUS "Maximum optimization for speed")
  add_compile_options(-Ofast)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
  message(STATUS "Maximum optimization for speed, debug info included")
  add_compile_options(-Ofast -g)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
  message(STATUS "Maximum optimization for size")
  add_compile_options(-Os)
else()
  message(STATUS "Minimal optimization, debug info included")
  add_compile_options(-Og -g)
endif()

project(Double_jointed_robotic_arm C CXX ASM)

# select board type
set(mcu "stm32f407") # C板
message(STATUS "MCU: ${mcu}")

# mcu subdirectory
add_subdirectory(mcu/${mcu})

if("${CMAKE_PRINT_COMPILE_OPTION}" STREQUAL "true")
  message(STATUS "definitions:\n   ${mcu_def}")
  message(STATUS "include_directories:\n   mcu/${mcu};")

  foreach(directory ${mcu_include})
    message("   mcu/${mcu}/${directory};")
  endforeach(directory)
endif()

add_definitions(${mcu_def})
include_directories(${CMAKE_SOURCE_DIR} mcu/${mcu})

foreach(directory ${mcu_include})
  include_directories(mcu/${mcu}/${directory})
endforeach(directory)

add_compile_options(${mcu_compile_options})
add_link_options(${mcu_link_options})

# library
file(GLOB_RECURSE libs "lib/*.a")

# submodules
add_subdirectory(base)
target_link_libraries(base ${libs} ${mcu}_driver)
add_subdirectory(app)
target_link_libraries(app ${libs} ${mcu}_driver base)
aux_source_directory(interface interface_src)
add_library(interface STATIC ${interface_src})
target_link_libraries(interface ${libs} ${mcu}_driver base app)

# executable file(.elf .hex .bin)
add_executable(${PROJECT_NAME}.elf ${mcu_src} ${interface_src} ${linker_scrpit})
target_link_libraries(${PROJECT_NAME}.elf ${libs} ${mcu}_driver interface)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
  COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
)
