# 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(MyLibrary)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 添加头文件目录
include_directories(include)

# 添加源文件
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 生成静态库
add_library(MyLibrary STATIC ${SOURCES})

# 安装库和头文件（可选）
install(TARGETS MyLibrary DESTINATION lib)
install(FILES include/msgpack.hpp include/msgpack-functions.hpp DESTINATION include)