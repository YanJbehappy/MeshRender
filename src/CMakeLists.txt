
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

add_subdirectory(Base)
add_subdirectory(PointCloud)

target_link_libraries(${PROJECT_NAME} LASlib base)

# target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GDAL_LIBRARY})
# target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS} ${Boost_headers_DIR} ${GDAL_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
)

#自定义文件拷贝等指令
add_custom_command(
  TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory
          "$<TARGET_FILE_DIR:${PROJECT_NAME}>/temp/"
  # COMMAND ${CMAKE_COMMAND} -E copy ${libLAS_dll}
  #         "$<TARGET_FILE_DIR:${PROJECT_NAME}>/"
  # COMMENT "Copying libLAS dll files to $<TARGET_FILE_DIR:${PROJECT_NAME}>/"
)