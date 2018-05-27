cmake_minimum_required (VERSION 2.8)
project(HelloSLAM)

IF(NOT CMAKE_BUILD_TYPE)
  #SET(CMAKE_BUILD_TYPE Release)
  #using debug view if need to Qt breakpoint. 
  SET(CMAKE_BUILD_TYPE "Debug")
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
LIST(APPEND CMAKE_MODULE_PATH "/home/poiwu/g2o/cmake_modules/")
set(G2O_DIR "/home/poiwu/g2o/cmake_modules")

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()


#include Eigen3/Pangolin/include contents
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED )


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/inc
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
${G2O_INCLUDE_DIRS}
)

#set uper contents to lib by using target_link_libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


add_library(${PROJECT_NAME} SHARED
src/UsingG2o.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${Sophus_LIBRARIES}
${EIGEN3_LIBS}
g2o_core g2o_stuff
)



#ch1
#add_executable(helloSLAM src/HelloSLAM.cpp)
#target_link_libraries(helloSLAM ${PROJECT_NAME})

#ch2
#add_executable(helloSLAM src/UsingEigen.cpp)
#target_link_libraries(helloSLAM ${PROJECT_NAME})

#ch3
#add_executable(helloSLAM src/UsingEigenTransforms.cpp)
#target_link_libraries(helloSLAM ${PROJECT_NAME})

#ch4
#add_executable(helloSLAM src/SophusTest.cpp)
#target_link_libraries(helloSLAM ${PROJECT_NAME})


#ch6
#add_executable(helloSLAM src/UsingG2o.cpp)
#target_link_libraries(helloSLAM ${PROJECT_NAME})

#ch Lucas_kanade
add_executable(helloSLAM src/L_K_opitcal.cpp)
target_link_libraries(helloSLAM ${PROJECT_NAME})
