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

option(RELEASE "Release build" OFF)

if (RELEASE)
	add_definitions(-DRELEASE)
endif(RELEASE)

link_directories(
  ${CMAKE_SOURCE_DIR}/../
)

add_executable(${PROJECT_NAME}.elf
	main.c
	log.c
)

target_link_libraries(${PROJECT_NAME}.elf
  taihen_stub
  SceLibKernel_stub
  SceIofilemgr_stub
  k
  gcc
  SceMotion_stub
  dsmotion_stub
)

set_target_properties(${PROJECT_NAME}.elf
  PROPERTIES LINK_FLAGS "-nostdlib"
)

vita_create_self(${PROJECT_NAME}.suprx ${PROJECT_NAME}.elf
  UNSAFE
  CONFIG ${CMAKE_SOURCE_DIR}/dsmotion.yml
)
