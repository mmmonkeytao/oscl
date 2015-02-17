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
