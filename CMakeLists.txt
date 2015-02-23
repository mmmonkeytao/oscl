cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# --------------------------------------------------------------
# Indicate CMake 2.8 and above that we don't want to mix relative
#  and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0042" for more information.
# --------------------------------------------------------------
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0042 NEW)
endif(COMMAND cmake_policy)

# online star clustering learning
project(oscl) 

set(OSCL_NAMESPACE oscl)

# Make sure stuff goes in to the correct places in the build dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

########################
# system configuration
########################
# MAC OS
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  add_definitions(-arch x86_64)
endif()

# Point CMake to the directory containing various build scripts
set(CMakeScripts_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH "${CMakeScripts_MODULE_DIR}")

# Bring in helper functions to make dealing
# with cache internal variables easier (functions from Dr.Rudolph)
include(CacheInternalHelpers)

# the variable contains all headers directories
# set(GLOABAL_INC_PATH)

#
set(OSCL_LIBRARIES CACHE INTERNAL
  "All individual libraries that go into the global one")


########################
# compiler settings
########################
# major working compiler clang++3.5
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
#set(CMAKE_CXX_FLAGS "-std=c++11 -DOPTIMIZE=1 -stdlib=libc++ -Wall -g -O3")
set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++ -Wall -g -O3")

# openmp/clang
# set(CMAKE_CXX_COMPILER "/Users/taoyeandy/code/llvm/build/Debug+Asserts/bin/clang2++")
# set(CMAKE_CXX_FLAGS "-std=c++11 -fopenmp -stdlib=libstdc++ -Wall -g -O3 -I/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk/usr/include/c++/4.2.1 -I/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk/usr/include/c++/4.2.1/backward -I/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/include -I/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk/usr/include -I/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/../lib/clang/6.0/include -I/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk/usr/include/c++/4.2.1/tr1/ -Duint=uint32_t"
#   )

# optional trial with bugs to be fixed
#set(CMAKE_CXX_COMPILER "g++-4.9")
#set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O3 -Duint=uint32_t")

########################
# Package Use 
########################
# QT
set(DESIRED_QT_VERSION 4)

# OpenCV
list(APPEND CMAKE_MODULE_PATH "/usr/local/Cellar/opencv/2.4.9/share/OpenCV")
find_package(OpenCV 2.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen  
set(EIGEN_INCLUDE_DIR "/usr/local/Cellar/eigen/3.2.2/include/eigen3/")
include_directories(${EIGEN_INCLUDE_DIR})

# boost
set(Boost_INCLUDE_DIR /usr/local/Cellar/boost/1.57.0/include)
set(Boost_LIBRARY_DIR /usr/local/Cellar/boost/1.57.0/lib)
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

# gsl
set(GSL_INCLUD_DIR "/usr/local/Cellar/gsl/1.16")
include_directories(${GSL_INCLUDE_DIR})

# stxxl
find_package(stxxl PATHS "./3dparty/stxxl-1.4.1/build")
include_directories(${STXXL_INCLUDE_DIRS})

# pcl
list(APPEND CMAKE_MODULE_PATH "/usr/local/Cellar/pcl/1.7.2/share/pcl-1.7/")
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# vtk
list(APPEND CMAKE_MODULE_PATH "/usr/local/Cellar/vtk/6.1.0/lib/cmake/vtk-6.1/")
find_package(VTK 6.1 REQUIRED)
include(${VTK_USE_FILE})


#########################
# add subdirectories 
#########################
add_subdirectory(engine)
add_subdirectory(hmp)
add_subdirectory(clustering)
add_subdirectory(lposc)
add_subdirectory(galaxy)
add_subdirectory(pcl)
add_subdirectory(test)

# We want to create dynamic libraries
set(BUILD_SHARED_LIBS true)

# Empty the export targets list
set(OSCL_TARGETS
  CACHE INTERNAL
  "Targets which should be exported"
)


#########################
# config script creation
#########################
# # Provide a cmake config script for the build directory
if (OSCL_LIBRARIES)
  list(REMOVE_DUPLICATES OSCL_LIBRARIES)
  export(TARGETS ${OSCL_LIBRARIES} FILE UseOSCL.cmake)
endif ()

# # Stick build location in the central CMake user package registry, so that it may be easily found
if ("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.8)
  export(PACKAGE OSCL)
endif()

#install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/hmp_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/hmp/")

#install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/hmp_rgbdeval" DESTINATION "${CMAKE_SOURCE_DIR}/app/hmp_rgbdeval/")

#install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/hmp_readfile" DESTINATION "${CMAKE_SOURCE_DIR}/app/hmp_readfile/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/clustering_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/clustering_test/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/openmp_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/openmp_test/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lposc_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/lposc_test/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/print_distr" DESTINATION "${CMAKE_SOURCE_DIR}/app/similarity_analysis/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/similarity_analysis" DESTINATION "${CMAKE_SOURCE_DIR}/app/similarity_analysis/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Adlposc_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/Adlposc/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/center_sim_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/center_sim/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/magnetic_osc" DESTINATION "${CMAKE_SOURCE_DIR}/app/magnetic_osc/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/loaded_insert_noSim_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/loaded_insert_noSim_test/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/guided_osc_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/guided_osc_test/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/normal_osc_insert_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/normal_osc_insert_test/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/pcl_visualizer" DESTINATION "${CMAKE_SOURCE_DIR}/app/pcl_visualizer/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/pcl_viewer" DESTINATION "${CMAKE_SOURCE_DIR}/app/pcl_viewer/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/convert_pcd2_dni" DESTINATION "${CMAKE_SOURCE_DIR}/app/convert_pcd2_dni/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/convert2DepNorm" DESTINATION "${CMAKE_SOURCE_DIR}/app/convert2DDepNorm/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/pcd2hmp" DESTINATION "${CMAKE_SOURCE_DIR}/app/pcd2hmp/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/kitti_sim_analyze" DESTINATION "${CMAKE_SOURCE_DIR}/app/kitti_sim_analyze/")

install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/kitti_osc_test" DESTINATION "${CMAKE_SOURCE_DIR}/app/kitti_osc_test/")

#install(PROGRAMS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/hmp_rgbdeval_full" DESTINATION "${CMAKE_SOURCE_DIR}/app/hmp_rgbdeval_full/")

#install(DIRECTORY ${ONLINECLUST_HMP_INCLUDE} DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/test/lib/inc")

#install(DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/test/lib/bin")

