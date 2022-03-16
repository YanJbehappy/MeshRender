#=============================================================================
# Copyright 2005-2011 Kitware, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# * Neither the name of Kitware, Inc. nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#=============================================================================

include(CMakeParseArguments)

# function里面的第一个参数是函数名，第二个为默认参数，ARGV和C++一样，ARGN表示默认参数以外的所有参数，例如：
# 定义一个宏，显式声明了两个参数hello,world
# macro(argn_test hello world)
# 	MESSAGE(STATUS ARGV=${ARGV})
# 	MESSAGE(STATUS ARGN=${ARGN})
# 	MESSAGE(STATUS ARGV0=${ARGV0})
# 	MESSAGE(STATUS ARGV1=${ARGV1})
# 	MESSAGE(STATUS ARGV2=${ARGV2})
# 	MESSAGE(STATUS ARGV3=${ARGV3})
# endmacro()
# ------调用宏时传入4个参数-------
# argn_test(TOM JERRY SUSAN BERN)
# ------输出------
# -- ARGV=TOMJERRYSUSANBERN
# -- ARGN=SUSANBERN
# -- ARGV0=TOM
# -- ARGV1=JERRY
# -- ARGV2=SUSAN
# -- ARGV3=BERN

function(QT5_CREATE_TRANSLATION_NO_INCLUDE _qm_files)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs OPTIONS)
    # cmake_parse_arguments 为解析函数（function）或 宏（macros） 参数的命令,最后一个${ARGN}便是解析的范围，可以还可以是${ARGC}等
    # 变量按输入参数(这里输入参数是${ARGN})里面包含的"${options}" "${oneValueArgs}" "${multiValueArgs}"字段去匹配
    # 此字段与下一个字段之间的参数即为此字段包含的所有参数
    # 将解析出来的值按下面_LUPDATE_参数名形成新的变量
    # "${options}" "${oneValueArgs}" "${multiValueArgs}"这几个字段第一个表示bool型变量，第二个表示单个值的变量，第三个表示可对应多个变量
    # 他们代表的含义与其命名无关而与他们在cmake_parse_arguments中的位置有关
    cmake_parse_arguments(_LUPDATE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    # _UNPARSED_ARGUMENTS表示未被使用的参数变量，即${ARGN}中没匹配到"${options}" "${oneValueArgs}" "${multiValueArgs}"字段的其余参数
    set(_lupdate_files ${_LUPDATE_UNPARSED_ARGUMENTS})
    set(_lupdate_options ${_LUPDATE_OPTIONS})

    set(_my_sources)
    set(_my_tsfiles)
    foreach(_file ${_lupdate_files})
        #从${_file}中获取关键字
        # 路径(PATH)，文件名(NAME)，文件扩展名(EXT)，去掉扩展名的文件名(NAME_WE)，完整路径(ABSOLUTE)
        # 或者所有符号链接被解析出的完整路径(REALPATH)。
        get_filename_component(_ext ${_file} EXT)
        get_filename_component(_abs_FILE ${_file} ABSOLUTE)
        if(_ext MATCHES "ts")
            list(APPEND _my_tsfiles ${_abs_FILE})
        else()
            list(APPEND _my_sources ${_abs_FILE})
        endif()
    endforeach()
    foreach(_ts_file ${_my_tsfiles})
        if(_my_sources)
          # make a list file to call lupdate on, so we don't make our commands too
          # long for some systems
          get_filename_component(_ts_name ${_ts_file} NAME_WE)
          set(_ts_lst_file "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${_ts_name}_lst_file")
          set(_lst_file_srcs)
          foreach(_lst_file_src ${_my_sources})
              set(_lst_file_srcs "${_lst_file_src}\n${_lst_file_srcs}")
          endforeach()

        #   get_directory_property(_inc_DIRS INCLUDE_DIRECTORIES)
        #   foreach(_pro_include ${_inc_DIRS})
        #       get_filename_component(_abs_include "${_pro_include}" ABSOLUTE)
        #       set(_lst_file_srcs "-I${_pro_include}\n${_lst_file_srcs}")
        #   endforeach()

          file(WRITE ${_ts_lst_file} "${_lst_file_srcs}")
        endif()
        add_custom_command(OUTPUT ${_ts_file}
            COMMAND ${Qt5_LUPDATE_EXECUTABLE}
            ARGS ${_lupdate_options} "@${_ts_lst_file}" -ts ${_ts_file}
            DEPENDS ${_my_sources} ${_ts_lst_file} VERBATIM)
    endforeach()
    qt5_add_translation(${_qm_files} ${_my_tsfiles})
    set(${_qm_files} ${${_qm_files}} PARENT_SCOPE)
endfunction()


function(QT5_ADD_TRANSLATION _qm_files)
    foreach(_current_FILE ${ARGN})
        get_filename_component(_abs_FILE ${_current_FILE} ABSOLUTE)
        get_filename_component(qm ${_abs_FILE} NAME_WE)
        get_source_file_property(output_location ${_abs_FILE} OUTPUT_LOCATION)
        if(output_location)
            file(MAKE_DIRECTORY "${output_location}")
            set(qm "${output_location}/${qm}.qm")
        else()
            set(qm "${CMAKE_CURRENT_BINARY_DIR}/${qm}.qm")
        endif()

        add_custom_command(OUTPUT ${qm}
            COMMAND ${Qt5_LRELEASE_EXECUTABLE}
            ARGS ${_abs_FILE} -qm ${qm}
            DEPENDS ${_abs_FILE} VERBATIM
        )
        list(APPEND ${_qm_files} ${qm})
    endforeach()
    set(${_qm_files} ${${_qm_files}} PARENT_SCOPE)
endfunction()
