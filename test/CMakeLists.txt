# hmp_test.cpp 
include_directories(${OSCL_HMP_INCLUDE})
include_directories(${OSCL_ENGINE_INCLUDE})
add_executable(hmp_test src/hmp_test.cpp)
target_link_libraries(hmp_test ${OSCL_HMP_LIBRARY} ${OSCL_ENGINE_LIBRARY})

#hmp_rgbdeval.cpp
add_executable(hmp_rgbdeval src/hmp_rgbdeval.cpp)
target_link_libraries(hmp_rgbdeval ${OSCL_HMP_LIBRARY} ${OSCL_ENGINE_LIBRARY})

#hmp_readfile.cpp
add_executable(hmp_readfile src/hmp_readfile.cpp)
target_link_libraries(hmp_readfile ${OSCL_HMP_LIBRARY} ${OSCL_ENGINE_LIBRARY})

# hmp_rgbdeval_full.cpp
add_executable(hmp_rgbdeval_full src/hmp_rgbdeval_full.cpp)
target_link_libraries(hmp_rgbdeval_full ${OSCL_HMP_LIBRARY} ${OSCL_ENGINE_LIBRARY})

# clustering_test.cpp
include_directories(${OSCL_CLUST_INCLUDE})
add_executable(clustering_test src/clustering_test.cpp)
target_link_libraries(clustering_test ${OSCL_CLUST_LIBRARY})

# lposc_test.cpp
include_directories(${OSCL_LPOSC_INCLUDE})
add_executable(lposc_test src/lposc_test.cpp)
target_link_libraries(lposc_test ${OSCL_LPOSC_LIBRARY})

# omp_test.cpp
add_executable(openmp_test src/openmp_test.cpp)

# print_distr.cpp
add_executable(print_distr src/print_distr.cpp)

# similarity_analysis
add_executable(similarity_analysis src/similarity_analysis.cpp)

# Adlposc_test.cpp
add_executable(Adlposc_test src/Adlposc_test.cpp)
target_link_libraries(Adlposc_test ${OSCL_LPOSC_LIBRARY})

# center_sim_test.cpp
add_executable(center_sim_test src/center_sim_test.cpp)
target_link_libraries(center_sim_test ${OSCL_LPOSC_LIBRARY})

# magnetic_osc.cpp
include_directories(${OSCL_GALAXY_INCLUDE})
add_executable(magnetic_osc src/magnetic_osc.cpp)
target_link_libraries(magnetic_osc ${OSCL_GALAXY_LIBRARY})

# loaded_insert_noSim_test.cpp
add_executable(loaded_insert_noSim_test src/loaded_insert_noSim_test.cpp)
target_link_libraries(loaded_insert_noSim_test ${OSCL_GALAXY_LIBRARY})

# guided_osc_test.cpp
add_executable(guided_osc_test src/guided_osc_test.cpp)
target_link_libraries(guided_osc_test ${OSCL_GALAXY_LIBRARY})

# normal_osc_test.cpp
add_executable(normal_osc_insert_test src/normal_osc_insert_test.cpp)
target_link_libraries(normal_osc_insert_test ${OSCL_GALAXY_LIBRARY})

# pcl_visualizer.cpp
add_executable(pcl_visualizer src/pcl_visualizer.cpp)
target_link_libraries(pcl_visualizer ${PCL_LIBRARIES})

# pcl_viewer.cpp
add_executable(pcl_viewer src/pcl_viewer.cpp)
target_link_libraries(pcl_viewer ${PCL_LIBRARIES}  ${Boost_PROGRAM_OPTIONS_LIBRARY})

# convert_pcd2_dni.cpp
include_directories(${OSCL_PCLOUD_INCLUDE})
add_executable(convert_pcd2_dni src/convert_pcd2_dni.cpp)
target_link_libraries(convert_pcd2_dni ${OSCL_PCLOUD_LIBRARY} ${PCL_LIBRARIES}  ${Boost_PROGRAM_OPTIONS_LIBRARY})

# convert2DepNorm.cpp
include_directories(${OSCL_PCLOUD_INCLUDE})
add_executable(convert2DepNorm src/convert2DepNorm.cpp)
target_link_libraries(convert2DepNorm ${OSCL_PCLOUD_LIBRARY} ${PCL_LIBRARIES}  ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OSCL_ENGINE_LIBRARY})

# pcd2hmp.cpp
add_executable(pcd2hmp src/pcd2hmp.cpp)
target_link_libraries(pcd2hmp ${OSCL_HMP_LIBRARY})


