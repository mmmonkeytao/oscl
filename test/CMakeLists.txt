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
#add_executable(openmp_test src/openmp_test.cpp)

