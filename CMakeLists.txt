cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

set(raisim_DIR /home/trec/WorkRaj/raisim_legged/raisim_build/lib/cmake/raisim)
set(OGRE_DIR /home/trec/WorkRaj/raisim_legged/raisim_build/OGRE/cmake)
set(raisimOgre_DIR /home/trec/WorkRaj/raisim_legged/raisim_build/share/raisimOgre/cmake)

find_package(Eigen3 REQUIRED)

find_package(raisim CONFIG HINTS "/home/trec/WorkRaj/raisim_legged/raisim_build/lib/cmake/raisim")
find_package(OGRE CONFIG REQUIRED PATHS "/home/trec/WorkRaj/raisim_legged/raisim_build/lib/OGRE/cmake")
find_package(raisimOgre CONFIG HINTS "/home/trec/WorkRaj/raisim_legged/raisim_build/share/raisimOgre/cmake")



include_directories (${EIGEN3_INCLUDE_DIRS})

add_executable(raisimExample main.cpp)
target_link_libraries(raisimExample PUBLIC raisim::raisim raisim::raisimOgre)

add_executable(PD_control_example pd_control.cpp)
target_link_libraries(PD_control_example PUBLIC raisim::raisim raisim::raisimOgre)

if(NOT WIN32)
  target_link_libraries(raisimExample PUBLIC pthread)
  target_link_libraries(PD_control_example PUBLIC pthread)
else()
  target_link_libraries(raisimExample PUBLIC Ws2_32 Winmm)
  target_link_libraries(PD_control_example PUBLIC Ws2_32 Winmm)
endif()

target_include_directories(raisimExample PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(PD_control_example PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
