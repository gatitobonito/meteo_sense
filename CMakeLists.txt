# Укажем под какую систему мы все нижеследующее будем собирать
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.17)
 
# Укажим какие компиляторы линковщики и т.д. будем использовать для кросс-компиляции
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
 
# Настройки проекта
project(METEO_SENSE C CXX ASM)
set(CMAKE_CXX_STANDART 17)
set(CMAKE_C_STANDART 11)
 
# Раскоментируйте ниже строчки для использования аппаратного FPU
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
 
add_compile_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
 
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
	message(STATUS "Maximum optimization for speed")
	add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
	message(STATUS "Maximum optimization for speed, debug info included")
	add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
	message(STATUS "Maximum optimization for size")
	add_compile_options(-Os)
else ()
	message(STATUS "Minimal optimization, debug info included")
	add_compile_options(-Og -g)
endif ()
 
# Тут прописываем пути ко всем инклудам проекта
include_directories(Core/Inc Drivers/STM32F1xx_HAL_Driver/Inc Drivers/STM32F1xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Include Drivers/CMSIS/Device)
 
# Тут прописываем дополнительные дефайны к проекту
add_definitions(-DUSE_HAL_DRIVER -DSTM32F103xB -DHSE_VALUE=16000000)
 
# Тут прописываются пути ко всем папкам с исходниками
file(GLOB_RECURSE SOURCES "Drivers/*.*" "Core/*.*")
 
# Тут указывается файл скрипта линковщика
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103CBTx_FLASH.ld)
 
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})
 
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
 
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
 
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
		COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
		COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")