file(GLOB HEADER "*.h" )
SOURCE_GROUP("header" FILES ${HEADER})
file(GLOB SOURCES "*.cpp")
SOURCE_GROUP("source" FILES ${SOURCES})

add_library(base ${SOURCES} ${HEADER})

set_target_properties(base PROPERTIES FOLDER "CloudViewer")

target_link_libraries(base Qt5::Core Qt5::Widgets Qt5::Gui Qt5::Charts pointcloud)