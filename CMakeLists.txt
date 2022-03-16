cmake_minimum_required(VERSION 3.5)

# 使用vcpkg添加第三方库
set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake") 

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})
# project(CloudViewer)

# init qt
set(CMAKE_AUTOMOC ON) # for meta object compiler
set(CMAKE_AUTORCC ON) # resource files
set(CMAKE_AUTOUIC ON) # UI files

#if中是忽略了常见的一些无用警告和bug
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /MP /wd4996 /wd4819 /wd4251 /wd4267 /wd4244 /wd4275 /wd4005 /wd5033 /wd4018")
    add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES -DWIN32_LEAN_AND_MEAN -DNOMINMAX -D_ENABLE_EXTENDED_ALIGNED_STORAGE) # windows use windows api
    # s.a https://cmake.org/cmake/help/latest/manual/cmake-generator-expressions.7.html
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
endif(MSVC)

# 在项目中启用文件夹目录
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# 将所有dll和exe放入bin目录下，将所有lib放入lib目录下。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_LIST_DIR}/cmake")

# find package
# find_package(Qt5 REQUIRED Widgets)
find_package(Qt5 COMPONENTS Core Gui Widgets Charts LinguistTools REQUIRED)
# include(Qt5LinguistToolsMacrosNoInclude)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED) # It's "Boost", not "BOOST" or "boost". Case matters.

# VTK相关target
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#添加矩阵运算库eigen
find_package(eigen3 REQUIRED)

# Laszip3::Laszip3
find_package(Laszip3 REQUIRED)

# GDAL库
find_package(GDAL REQUIRED)

#添加头文件搜索目录 Eigen3头文件目录 当前cmakelists文件所在目录/src
include_directories(${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src)
# 添加根目录下的第三方库
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libLAS")
# set(libLAS_headers "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libLAS/include")
# set(libLAS_dll_dir "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libLAS/dll")
# set(libLAS_lib_dir "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libLAS/lib")

set(kd_tree_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/kd_tree")
set(cminpack_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/cminpack")
set(lm_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/optimizer_lm")
set(easy3d_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/easy3d")
set(tetgen_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/tetgen")

add_subdirectory(src)
# add_subdirectory(3rd_party)
add_subdirectory(LASlib)
