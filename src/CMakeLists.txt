
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

file(GLOB DIR_SRCS    "*.cpp")
file(GLOB DIR_HEADERS "*.h")
file(GLOB DIR_UIS     "*.ui")

# add resource
set(RESOURCE ${CMAKE_CURRENT_SOURCE_DIR}/CloudViewer.qrc)
qt5_add_resources(RESOURCE_ADDED ${RESOURCE})

add_executable(${PROJECT_NAME}
    ${DIR_SRCS}
    ${DIR_HEADERS}
    ${DIR_UIS}
    ${RESOURCE_ADDED}
)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "CloudViewer")

add_subdirectory(IO)
add_subdirectory(Base)
add_subdirectory(PointCloud)

# 链接第三方库
# target_include_directories(${PROJECT_NAME} PRIVATE ${kd_tree_INCLUDE_DIR})
# target_include_directories(${PROJECT_NAME} PRIVATE ${lm_INCLUDE_DIR})
# target_include_directories(${PROJECT_NAME} PRIVATE ${easy3d_INCLUDE_DIR})
# target_include_directories(${PROJECT_NAME} PRIVATE ${tetgen_INCLUDE_DIR})

# target_link_libraries(${PROJECT_NAME} cminpack optimizer_lm kd_tree core tetgen LASlib io base)
target_link_libraries(${PROJECT_NAME} LASlib io base)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GDAL_LIBRARY})
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS} ${Boost_headers_DIR} ${GDAL_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
)

# file(GLOB libLAS_dll ${libLAS_dll_dir}/*.dll)
# file(GLOB libLAS_lib ${libLAS_lib_dir}/*.lib)

# target_include_directories(${PROJECT_NAME} PRIVATE ${libLAS_headers})
# target_link_libraries(${PROJECT_NAME} ${libLAS_lib})

#自定义文件拷贝等指令
add_custom_command(
  TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory
          "$<TARGET_FILE_DIR:${PROJECT_NAME}>/temp/"
  # COMMAND ${CMAKE_COMMAND} -E copy ${libLAS_dll}
  #         "$<TARGET_FILE_DIR:${PROJECT_NAME}>/"
  # COMMENT "Copying libLAS dll files to $<TARGET_FILE_DIR:${PROJECT_NAME}>/"
)