cmake_minimum_required(VERSION 2.8.3)
project(pcsdk)

set(PCSDK_DIR "PCSDK")

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation roscpp rostime rosconsole roscpp_serialization)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCSDK_DIR}/include
  ${PCSDK_DIR}/sample/common/include
  ${PCSDK_DIR}/sample/common/res
)
link_directories(${catkin_LIBRARY_DIRS})

add_service_files(FILES Grammar.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMINGW_HAS_SECURE_API=1")

set(T "recog")
add_executable(${T}
  src/recog.cc
  ${PCSDK_DIR}/sample/common/src/util_capture.cpp
  ${PCSDK_DIR}/src/libpxc/libpxc.cpp
)
set_source_files_properties(${PCSDK_DIR}/sample/common/src/util_capture.cpp PROPERTIES COMPILE_FLAGS "-include cstring")
set_source_files_properties(${PCSDK_DIR}/src/libpxc/libpxc.cpp PROPERTIES COMPILE_FLAGS "-D_UNICODE -DUNICODE")
target_link_libraries(${T} ${catkin_LIBRARIES} winmm)
add_dependencies(${T} ${PROJECT_NAME}_gencpp)
add_custom_command(TARGET ${T} POST_BUILD COMMAND ${CMAKE_STRIP} $<TARGET_FILE:${T}>)
install(TARGETS ${T} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
