file(GLOB HEADER "*.h")
file(GLOB SOURCE "*.cpp")
SOURCE_GROUP("header" FILES ${HEADER})
SOURCE_GROUP("source" FILES ${SOURCE})

add_library(io ${HEADER} ${SOURCE})

set_target_properties(io PROPERTIES FOLDER "CloudViewer")

target_link_libraries(io Laszip3::Laszip3 Qt5::Core pointcloud)