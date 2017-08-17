cmake_minimum_required(VERSION 2.8)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake" CACHE PATH "toolchain file")
  else()
    message(FATAL_ERROR "Please define VITASDK to point to your SDK path!")
  endif()
endif()

project(dsmotion)
include("${VITASDK}/share/vita.cmake" REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,-q -Wall -O3 -nostdlib")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions")

add_executable(${PROJECT_NAME}.elf
	main.c
)

target_link_libraries(${PROJECT_NAME}.elf
	gcc
	taihenForKernel_stub
    SceThreadmgrForDriver_stub
    SceModulemgrForKernel_stub
	SceSysclibForDriver_stub
	SceSysmemForDriver_stub
	SceSysmemForKernel_stub
	SceIofilemgrForDriver_stub
	SceBtForDriver_stub
	SceKernelSuspendForDriver_stub
)

set_target_properties(${PROJECT_NAME}.elf
	PROPERTIES LINK_FLAGS "-nostdlib"
	COMPILE_FLAGS "-D__VITA_KERNEL__"
)

add_custom_target(${PROJECT_NAME}.skprx ALL
	COMMAND vita-elf-create -e ${PROJECT_NAME}.yml ${PROJECT_NAME}.elf ${PROJECT_NAME}.velf
	COMMAND vita-make-fself -c ${PROJECT_NAME}.velf ${PROJECT_NAME}.skprx
)
add_dependencies(${PROJECT_NAME}.skprx ${PROJECT_NAME}.elf)

vita_create_stubs(stubs ${PROJECT_NAME}.elf ${PROJECT_NAME}.yml KERNEL)

install(DIRECTORY ${CMAKE_BINARY_DIR}/stubs/
  DESTINATION ${CMAKE_BINARY_DIR}/../
  FILES_MATCHING PATTERN "*.a"
)
