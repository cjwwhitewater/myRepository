# 定义通用查找函数
function(find_and_add_library name header header_path libnames lib_paths)
    find_path(${name}_INCLUDE_DIR ${header} PATHS ${header_path})
    find_library(${name}_LIB ${libnames} PATHS ${lib_paths})
    if(${name}_INCLUDE_DIR AND ${name}_LIB)
        message(STATUS "Found ${name}: ${${name}_LIB}")
        include_directories(${${name}_INCLUDE_DIR})
        list(APPEND PROJECT_LIBS ${${name}_LIB})
        set(PROJECT_LIBS ${PROJECT_LIBS} PARENT_SCOPE)
    else()
        message(FATAL_ERROR "${name} not found in sysroot!")
    endif()
endfunction()

# 通用头文件和库路径
set(SYSROOT_INCLUDE_DIRS
    ${CMAKE_SYSROOT}/usr/include
    ${CMAKE_SYSROOT}/usr/local/include
    ${CMAKE_SYSROOT}/include
    ${CMAKE_SYSROOT}/usrLib
    ${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu/
)
set(SYSROOT_LIB_DIRS
    ${CMAKE_SYSROOT}/usr/lib
    ${CMAKE_SYSROOT}/usr/local/lib
    ${CMAKE_SYSROOT}/lib/aarch64-linux-gnu
    ${CMAKE_SYSROOT}/lib
    ${CMAKE_SYSROOT}/usrLib
    ${CMAKE_SYSROOT}/lib/aarch64-linux-gnu/tegra
    ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu
)

set(PSDK_INCLUDE_DIR "${CMAKE_SYSROOT}/usr/local/lib/psdk/include")
set(PSDK_LIB_DIR "${CMAKE_SYSROOT}/usr/local/lib/psdk/lib/aarch64-linux-gnu-gcc")

# jsoncpp
find_and_add_library(JSONCPP jsoncpp/json/json.h "${SYSROOT_INCLUDE_DIRS}" jsoncpp "${SYSROOT_LIB_DIRS}")

# jetgpio
find_and_add_library(JETGPIO jetgpio.h "${SYSROOT_INCLUDE_DIRS}" jetgpio "${SYSROOT_LIB_DIRS}")

# options
find_and_add_library(OPTIONS options/options.h "${SYSROOT_INCLUDE_DIRS}" options "${SYSROOT_LIB_DIRS}")
include_directories(${OPTIONS_INCLUDE_DIR}/options)

#psdk
find_library(PAYLOADSDK_LIB payloadsdk PATHS ${PSDK_LIB_DIR})

if(PAYLOADSDK_LIB)
    message(STATUS "Found payloadsdk: ${PAYLOADSDK_LIB}")
    list(APPEND PROJECT_LIBS ${PAYLOADSDK_LIB})
else()
    message(FATAL_ERROR "payloadsdk not found in sysroot!")
endif()

include_directories(${PSDK_INCLUDE_DIR})

# 项目头文件
include_directories(${CMAKE_SOURCE_DIR}/include)

# 链接全部第三方库
set(ALL_THIRD_PARTY_LIBS
    ${PROJECT_LIBS}
    pthread
    m
)

# sysroot下的库目录
link_directories(${SYSROOT_LIB_DIRS})

